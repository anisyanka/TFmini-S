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

union tf_mini_s_rxdata {
    uint8_t arr[TF_FRAME_PAYLOAD_SIZE + TF_FRAME_CRC_SIZE];
    struct {
        uint16_t distance;
        uint16_t strength;
        uint16_t raw_temp;
        uint8_t crc;
    } fields;
};

typedef struct {
    void *itself;
    union tf_mini_s_rxdata rxdata;
    int cur_byte_idx;
    int sof_cnt;
    int dev_number;
    parser_state_t parser_state;
    uint8_t crc;
} _private_tfnminis_data_t;

static _private_tfnminis_data_t _private[TFMINIS_DEVICES_IN_SYSTEM] = { 0 };
static int _dev_cnt = 0;

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
    int dev_exists_flag = 0;

    if (check_args(dev) != TFMINIS_OK) {
        return TFMINIS_WRONG_ARGS;
    }

    /* Stop data obtaining to use polling to setup the sensor */
    if (dev->ll->stop_dma_or_irq_operations()) {
        return TFMINIS_INTERF_ERR;
    }

    /* Does local structure for this device exist? */
    for (int i = 0; i < TFMINIS_DEVICES_IN_SYSTEM; ++i) {
        if (_private[i].itself == dev) {
            dev_exists_flag = 1;
        }
    }

    /* new device */
    if (dev_exists_flag == 0) {
        if (_dev_cnt >= TFMINIS_DEVICES_IN_SYSTEM) {
            /* we can't create more devices then TFMINIS_DEVICES_IN_SYSTEM. Set the parametr in your build system */
            return TFMINIS_WRONG_ARGS;
        }

        /* Init a new private data the for new device */
        _private[_dev_cnt].itself = dev;
        _private[_dev_cnt].dev_number = _dev_cnt;
        _private[_dev_cnt].cur_byte_idx = 0;
        _private[_dev_cnt].sof_cnt = 0;
        _private[_dev_cnt].parser_state = WAIT_FOR_SOF;
        _private[_dev_cnt].crc = 0;
        dev->_private = &_private[_dev_cnt];

        ++_dev_cnt;
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

static void tfminis_data_parser(tfminis_dev_t *dev, uint8_t byte)
{
    /* get local private struct */
    _private_tfnminis_data_t *_private_d = (_private_tfnminis_data_t *)dev->_private;

    /* Detect start of frame */
    if (byte == TF_FRAME_START_BYTE && _private_d->parser_state == WAIT_FOR_SOF) {
        ++_private_d->sof_cnt;

        if (_private_d->sof_cnt == TF_FRAME_START_BYTES_CNT) {
            _private_d->crc += TF_FRAME_START_BYTE;
            _private_d->crc += TF_FRAME_START_BYTE;
            _private_d->parser_state = WAIT_FOR_DIST_L;
        }
        return;
    }

    /* If we got here we received TF_FRAME_START_BYTE one times. It means frame is broken */
    if (_private_d->sof_cnt == TF_FRAME_START_BYTES_CNT - 1) {
        _private_d->crc = 0;
        _private_d->cur_byte_idx = 0;
        _private_d->sof_cnt = 0;
        _private_d->parser_state = WAIT_FOR_SOF;
        return;
    }

    /* save usefull data */
    switch (_private_d->parser_state) {
    case WAIT_FOR_DIST_L:
        _private_d->crc += byte;
        _private_d->rxdata.arr[_private_d->cur_byte_idx++] = byte;
        _private_d->parser_state = WAIT_FOR_DIST_H;
        break;
    case WAIT_FOR_DIST_H:
        _private_d->crc += byte;
        _private_d->rxdata.arr[_private_d->cur_byte_idx++] = byte;
        _private_d->parser_state = WAIT_FOR_STRENGTH_L;
        break;
    case WAIT_FOR_STRENGTH_L:
        _private_d->crc += byte;
        _private_d->rxdata.arr[_private_d->cur_byte_idx++] = byte;
        _private_d->parser_state = WAIT_FOR_STRENGTH_H;
        break;
    case WAIT_FOR_STRENGTH_H:
        _private_d->crc += byte;
        _private_d->rxdata.arr[_private_d->cur_byte_idx++] = byte;
        fill_dist_data_based(dev, _private_d->rxdata.fields.distance, _private_d->rxdata.fields.strength);
        _private_d->parser_state = WAIT_FOR_TEMP_L;
        break;
    case WAIT_FOR_TEMP_L:
        _private_d->crc += byte;
        _private_d->rxdata.arr[_private_d->cur_byte_idx++] = byte;
        _private_d->parser_state = WAIT_FOR_TEMP_H;
        break;
    case WAIT_FOR_TEMP_H:
        _private_d->crc += byte;
        _private_d->rxdata.arr[_private_d->cur_byte_idx++] = byte;
        dev->temperature_c = (_private_d->rxdata.fields.raw_temp / 8) - 256;
        _private_d->parser_state = WAIT_FOR_CRC;
        break;
    case WAIT_FOR_CRC:
        _private_d->rxdata.arr[_private_d->cur_byte_idx++] = byte;
        if (_private_d->crc != byte) {
            dev->dist.err_reason = TFMINIS_CRC_FAILED;
        }
        break;
    default:
        break;
    }

    /* Obtained whole frame or prevent rx buffer overflow in case of broken frames */
    if (_private_d->cur_byte_idx > sizeof(_private_d->rxdata.arr) - 1) {
        if (dev->ll->data_avaliabe_isr_cb) { /* if not null */
            dev->ll->data_avaliabe_isr_cb();
        }

        _private_d->crc = 0;
        _private_d->cur_byte_idx = 0;
        _private_d->sof_cnt = 0;
        _private_d->parser_state = WAIT_FOR_SOF;
    }
}

void tfminis_handle_rx_byte_uart_isr(tfminis_dev_t *dev, uint8_t byte)
{
    tfminis_data_parser(dev, byte);
}

void tfminis_handle_rx_data_dma_isr(tfminis_dev_t *dev, uint8_t *rx_frame, size_t len)
{
    for (int i = 0; i < len; ++i) {
        tfminis_data_parser(dev, rx_frame[i]);
    }
}
