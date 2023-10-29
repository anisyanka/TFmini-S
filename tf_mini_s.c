#include "tf_mini_s.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define TF_FRAME_SIZE (9U)
#define TF_FRAME_PAYLOAD_SIZE (6U)
#define TF_FRAME_CRC_SIZE (1U)
#define TF_FRAME_START_BYTE (0x59)
#define TF_FRAME_START_BYTES_CNT (2)

#define TF_COMMAND_OUT_DISABLE_ATTEMPTS (5U)

#define TF_DIST_TARGET_TOO_FAR (65535U)
#define TF_STRENGHT_TARGET_TOO_FAR (100U)

#define TF_DIST_TARGET_TOO_CLOSE (65534U)
#define TF_STRENGHT_TARGET_TOO_CLOSE (65535U)

#define TF_DIST_AMBIENT_SATURATION (65532U)

union tf_mini_s_rxdata {
    uint8_t arr[TF_FRAME_PAYLOAD_SIZE + TF_FRAME_CRC_SIZE];
    struct {
        uint16_t distance;
        uint16_t strength;
        uint16_t raw_temp;
        uint8_t crc;
    } fields;
};

static tfminis_ret_t check_args(tfminis_dev_t *dev)
{
    if (!dev || !dev->ll || !dev->ll->uart_send || \
        !dev->ll->uart_recv || !dev->ll->delay_ms || \
        !dev->ll->stop_dma_or_irq_operations || \
        !dev->ll->start_dma_or_irq_operations) {
        return TFMINIS_WRONG_ARGS;
    }

    return TFMINIS_OK;
}

static uint8_t calc_crc(uint8_t *d, size_t len)
{
    uint8_t checksum = 0;

    for (int i = 0; i < len; ++i) {
        checksum += d[i];
    }

    return checksum;
}

static tfminis_ret_t disable_data_frames_output(tfminis_dev_t *dev)
{
    tfminis_ret_t ret;

    int attempts = 0; /* try to send it several times to prevent intervention of previous data frame */
    uint8_t command[] = { 0x5A, 0x05, 0x07, 0x00, 0x66 };
    uint8_t response[sizeof(command)] = { 0 };

    while (attempts++ < TF_COMMAND_OUT_DISABLE_ATTEMPTS) {
        ret = dev->ll->uart_send(command, sizeof(command));
        ret |= dev->ll->uart_recv(response, sizeof(response));
        if (ret) {
            return TFMINIS_INTERF_ERR;
        } else if ((memcmp(command, response, sizeof(command)) != 0) && (attempts == TF_COMMAND_OUT_DISABLE_ATTEMPTS)) {
            return TFMINIS_WRONG_RESPONSE;
        } else {
            break;
        }

        dev->ll->delay_ms(20);
    }

    return TFMINIS_OK;
}

static tfminis_ret_t enable_data_frames_output(tfminis_dev_t *dev)
{
    tfminis_ret_t ret;

    uint8_t command[] = { 0x5A, 0x05, 0x07, 0x01, 0x67 };
    uint8_t response[sizeof(command)] = { 0 };

    ret = dev->ll->uart_send(command, sizeof(command));
    ret |= dev->ll->uart_recv(response, sizeof(response));
    if (ret) {
        return TFMINIS_INTERF_ERR;
    }

    if (memcmp(command, response, sizeof(command)) != 0) {
        return TFMINIS_WRONG_RESPONSE;
    }

    return TFMINIS_OK;
}

tfminis_ret_t tfminis_init(tfminis_dev_t *dev, tfminis_interfaces_t interf)
{
    tfminis_ret_t ret;

    if (check_args(dev) != TFMINIS_OK) {
        return TFMINIS_WRONG_ARGS;
    }

    /* Stop data obtaining to use polling to setup the sensor */
    if (dev->ll->stop_dma_or_irq_operations()) {
        return TFMINIS_INTERF_ERR;
    }

    /* delay after power up to wait the sensor init process */
    dev->ll->delay_ms(1000);

    ret = disable_data_frames_output(dev);
    if (ret != TFMINIS_OK) {
        return ret;
    }

    dev->ll->delay_ms(20);
    dev->interface_type = interf; /* for future use */

    /* Get FW version */
    {
        uint8_t command[4] = { 0x5A, 0x04, 0x01, 0x5F };
        uint8_t response[7] = { 0 };
        uint8_t checksum = 0;

        ret = dev->ll->uart_send(command, sizeof(command));
        ret |= dev->ll->uart_recv(response, sizeof(response));
        if (ret) {
            return TFMINIS_INTERF_ERR;
        }

        if (response[0] != 0x5A || response[1] != 0x07 || response[2] != 0x01) {
            return TFMINIS_WRONG_RESPONSE;
        }

        checksum = calc_crc(response, sizeof(response) - 1);
        if (checksum != response[sizeof(response) - 1]) {
            return TFMINIS_WRONG_CRC;
        }

        /* [3]=V1, [4]=V2, [5]=V3 */
        dev->fw_version = (uint32_t)response[5] | ((uint32_t)response[4] << 8) | ((uint32_t)response[3] << 16);
    }

    ret = enable_data_frames_output(dev);
    if (ret != TFMINIS_OK) {
        return ret;
    }

    /* Start data obtaining in async mode */
    if (dev->ll->start_dma_or_irq_operations()) {
        return TFMINIS_FAIL;
    }

    return TFMINIS_OK;
}

tfminis_dist_t tfminis_get_distance(tfminis_dev_t *dev)
{
    return dev->dist;
}

tfminis_dist_error_reason_t tfminis_get_err_reason(tfminis_dev_t *dev)
{
    return dev->dist.err_reason;
}

uint16_t tfminis_get_chip_temp(tfminis_dev_t *dev)
{
    return dev->temperature_c;
}

tfminis_ret_t tfminis_set_frame_rate(tfminis_dev_t *dev, uint16_t frame_rate)
{
    tfminis_ret_t ret;

    uint8_t command[] = { 0x5A, 0x06, 0x03, 0x00, 0x00, 0x00 };
    uint8_t response[sizeof(command)];

    if (frame_rate > 1000) {
        return TFMINIS_WRONG_ARGS;
    }

    if (dev->ll->stop_dma_or_irq_operations()) {
        return TFMINIS_INTERF_ERR;
    }

    ret = disable_data_frames_output(dev);
    if (ret != TFMINIS_OK) {
        return ret;
    }

    command[3] = (uint8_t) (frame_rate & 0x00ff); /* low byte */
    command[4] = (uint8_t) (frame_rate >> 8); /* high byte */
    command[sizeof(command) - 1] = calc_crc(command, sizeof(command) - 1);

    ret = dev->ll->uart_send(command, sizeof(command));
    ret |= dev->ll->uart_recv(response, sizeof(response));
    if (ret) {
        dev->ll->start_dma_or_irq_operations();
        return TFMINIS_INTERF_ERR;
    }

    if (memcmp(command, response, sizeof(command)) != 0) {
        dev->ll->start_dma_or_irq_operations();
        return TFMINIS_WRONG_RESPONSE;
    }

    ret = enable_data_frames_output(dev);
    if (ret != TFMINIS_OK) {
        return ret;
    }

    /* Start data obtaining in async mode */
    if (dev->ll->start_dma_or_irq_operations()) {
        return TFMINIS_INTERF_ERR;
    }

    return TFMINIS_OK;
}

tfminis_ret_t tfminis_get_distance_oneshot(tfminis_dev_t *dev, tfminis_dist_t *return_dist)
{
    tfminis_ret_t ret;
    uint8_t command[4] = { 0x5A, 0x04, 0x04, 0x62 };
    uint8_t response[TF_FRAME_SIZE] = { 0 };
    uint8_t crc = 0;
    uint16_t distance = 0;
    uint16_t strength = 0;

    ret = dev->ll->uart_send(command, sizeof(command));
    ret |= dev->ll->uart_recv(response, sizeof(response));

    distance = (uint16_t)response[3] << 8 | response[2];
    strength = (uint16_t)response[5] << 8 | response[4];
    dev->temperature_c = (((uint16_t)response[7] << 8 | response[6]) / 8 - 256);

    return_dist->distance_cm = distance;
    return_dist->strength = strength;

    if (ret) {
        return_dist->err_reason = TFMINIS_INTF_ERR;
        return TFMINIS_FAIL;
    }

    if (response[0] != TF_FRAME_START_BYTE || response[1] != TF_FRAME_START_BYTE) {
        return_dist->err_reason = TFMINIS_NOT_A_DATA_FRAME;
        return TFMINIS_FAIL;
    }

    crc = calc_crc(response, sizeof(response) - 1);
    if (crc != response[sizeof(response) - 1]) {
        return_dist->err_reason = TFMINIS_CRC_FAILED;
        return TFMINIS_FAIL;
    }

    /* distance more than max operating range */
    if (distance == TF_DIST_TARGET_TOO_FAR && strength < TF_STRENGHT_TARGET_TOO_FAR) {
        return_dist->err_reason = TFMINIS_TOO_FAR_TARGET;
        return TFMINIS_FAIL;
    }

    /* too good reflectivity */
    if (distance == TF_DIST_TARGET_TOO_CLOSE && strength == TF_STRENGHT_TARGET_TOO_CLOSE) {
        return_dist->err_reason = TFMINIS_STRENGTH_SATURATION;
        return TFMINIS_FAIL;
    }

    /* too much ambient light */
    if (distance == TF_DIST_AMBIENT_SATURATION) {
        return_dist->err_reason = TFMINIS_STRENGTH_SATURATION;
        return TFMINIS_FAIL;
    }

    return_dist->err_reason = TFMINIS_DATA_IS_VALID;
    return TFMINIS_OK;
}

typedef enum {
    WAIT_FOR_SOF,
    WAIT_FOR_DIST_L,
    WAIT_FOR_DIST_H,
    WAIT_FOR_STRENGTH_L,
    WAIT_FOR_STRENGTH_H,
    WAIT_FOR_TEMP_L,
    WAIT_FOR_TEMP_H,
    WAIT_FOR_CRC,
} parser_state_t;

static void fill_dist_data_based(tfminis_dev_t *dev, uint16_t distance, uint16_t strength)
{
    dev->dist.distance_cm = distance;
    dev->dist.strength = strength;

    /* distance more than max operating range */
    if (distance == TF_DIST_TARGET_TOO_FAR && strength < TF_STRENGHT_TARGET_TOO_FAR) {
        dev->dist.err_reason = TFMINIS_TOO_FAR_TARGET;
        return;
    }

    /* distance less than min operating range */
    if (distance == TF_DIST_TARGET_TOO_CLOSE && strength == TF_STRENGHT_TARGET_TOO_CLOSE) {
        dev->dist.err_reason = TFMINIS_STRENGTH_SATURATION;
        return;
    }

    /* too much ambient light */
    if (distance == TF_DIST_AMBIENT_SATURATION) {
        dev->dist.err_reason = TFMINIS_STRENGTH_SATURATION;
        return;
    }

    dev->dist.err_reason = TFMINIS_DATA_IS_VALID;
}

void tfminis_handle_rx_byte_uart_isr(tfminis_dev_t *dev, uint8_t byte)
{
    static union tf_mini_s_rxdata rxdata;
    static int cur_byte_idx = 0;
    static int sof_cnt = 0;
    static parser_state_t parser_state = WAIT_FOR_SOF;
    static uint8_t crc = 0;

    /* Detect start of frame */
    if (byte == TF_FRAME_START_BYTE && parser_state == WAIT_FOR_SOF) {
        ++sof_cnt;

        if (sof_cnt == TF_FRAME_START_BYTES_CNT) {
            crc += TF_FRAME_START_BYTE;
            crc += TF_FRAME_START_BYTE;
            parser_state = WAIT_FOR_DIST_L;
        }
        return;
    }

    /* If we got here we received TF_FRAME_START_BYTE one times. It means frame is broken */
    if (sof_cnt == TF_FRAME_START_BYTES_CNT - 1) {
        crc = 0;
        cur_byte_idx = 0;
        sof_cnt = 0;
        parser_state = WAIT_FOR_SOF;
        return;
    }

    /* save usefull data */
    switch (parser_state) {
    case WAIT_FOR_DIST_L:
        crc += byte;
        rxdata.arr[cur_byte_idx++] = byte;
        parser_state = WAIT_FOR_DIST_H;
        break;
    case WAIT_FOR_DIST_H:
        crc += byte;
        rxdata.arr[cur_byte_idx++] = byte;
        parser_state = WAIT_FOR_STRENGTH_L;
        break;
    case WAIT_FOR_STRENGTH_L:
        crc += byte;
        rxdata.arr[cur_byte_idx++] = byte;
        parser_state = WAIT_FOR_STRENGTH_H;
        break;
    case WAIT_FOR_STRENGTH_H:
        crc += byte;
        rxdata.arr[cur_byte_idx++] = byte;
        fill_dist_data_based(dev, rxdata.fields.distance, rxdata.fields.strength);
        parser_state = WAIT_FOR_TEMP_L;
        break;
    case WAIT_FOR_TEMP_L:
        crc += byte;
        rxdata.arr[cur_byte_idx++] = byte;
        parser_state = WAIT_FOR_TEMP_H;
        break;
    case WAIT_FOR_TEMP_H:
        crc += byte;
        rxdata.arr[cur_byte_idx++] = byte;
        dev->temperature_c = (rxdata.fields.raw_temp / 8) - 256;
        parser_state = WAIT_FOR_CRC;
        break;
    case WAIT_FOR_CRC:
        rxdata.arr[cur_byte_idx++] = byte;
        if (crc != byte) {
            dev->dist.err_reason = TFMINIS_CRC_FAILED;
        }
        break;
    default:
        break;
    }

    /* Obtained whole frame or prevent rx buffer overflow in case of broken frames */
    if (cur_byte_idx > sizeof(rxdata.arr) - 1) {
        if (dev->ll->data_avaliabe_isr_cb) { /* if not null */
            dev->ll->data_avaliabe_isr_cb();
        }

        crc = 0;
        cur_byte_idx = 0;
        sof_cnt = 0;
        parser_state = WAIT_FOR_SOF;
    }
}
