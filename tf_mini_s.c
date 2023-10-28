#include "tf_mini_s.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define TF_FRAME_SIZE (9U)
#define TF_FRAME_PAYLOAD_SIZE (6U)
#define TF_FRAME_START_BYTE (0x59)

#define TF_COMMAND_FRAME_ATTEMPTS (3U)

#define TF_DIST_TARGET_TOO_FAR (65535U)
#define TF_STRENGHT_TARGET_TOO_FAR (100U)

#define TF_DIST_TARGET_TOO_CLOSE (65534U)
#define TF_STRENGHT_TARGET_TOO_CLOSE (65535U)

#define TF_DIST_AMBIENT_SATURATION (65532U)

#define TF_STRENGHT_PROB_CONVERTION_RANGE_COEFF (655) /* 1/(100/65530) */

union tf_mini_s_rxdata {
    uint8_t arr[TF_FRAME_PAYLOAD_SIZE];
    struct {
        uint16_t distance;
        uint16_t strength;
        uint16_t raw_temp;
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

tfminis_ret_t tfminis_init(tfminis_dev_t *dev, tfminis_interfaces_t interf)
{
    tfminis_ret_t ret;

    if (check_args(dev) != TFMINIS_OK) {
        return TFMINIS_WRONG_ARGS;
    }

    /* delay after power up to wait the sensor init process */
    dev->ll->delay_ms(1000);

    /* Stop data obtaining to use polling to setup the sensor */
    if (dev->ll->stop_dma_or_irq_operations()) {
        return TFMINIS_INTERF_ERR;
    }

    /* Set frame rate to 0 to stop data frames */
    {
        int attempts = 0; /* try to send it several times to prevent intervention of previous data frame */
        uint8_t command[] = { 0x5A, 0x06, 0x03, 0x00, 0x00, 0x63 };
        uint8_t response[sizeof(command)] = { 0 };

        while (attempts++ < TF_COMMAND_FRAME_ATTEMPTS) {
            ret = dev->ll->uart_send(command, sizeof(command));
            ret |= dev->ll->uart_recv(response, sizeof(response));

            if (ret && (attempts == TF_COMMAND_FRAME_ATTEMPTS)) {
                return TFMINIS_INTERF_ERR;
            } else if ((memcmp(command, response, sizeof(command)) != 0) && (attempts == TF_COMMAND_FRAME_ATTEMPTS)) {
                return TFMINIS_WRONG_RESPONSE;
            } else {
                break;
            }

            dev->ll->delay_ms(20);
        }
    }

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

    /* Enable data frames output */
    {
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
    }

    /* Start data obtaining in async mode */
    if (dev->ll->start_dma_or_irq_operations()) {
        return TFMINIS_FAIL;
    }

    return TFMINIS_OK;
}

tfminis_dist_t tfminis_get_distance(tfminis_dev_t *dev)
{
    tfminis_dist_t d;
    return d;
}

tfminis_dist_error_reason_t tfminis_get_err_reason(tfminis_dev_t *dev)
{
    return TFMINIS_OK;
}

uint16_t tfminis_get_chip_temp(tfminis_dev_t *dev)
{
    return TFMINIS_OK;
}

tfminis_ret_t tfminis_set_frame_rate(tfminis_dev_t *dev)
{
    return TFMINIS_OK;
}

tfminis_ret_t tfminis_get_distance_oneshot(tfminis_dev_t *dev, tfminis_dist_t *return_dist)
{
    tfminis_ret_t ret;
    uint8_t command[4] = { 0x5A, 0x04, 0x04, 0x62 };
    uint8_t response[TF_FRAME_SIZE] = { 0 };
    uint8_t crc = 0;
    uint16_t distanse = 0;
    uint16_t strength = 0;

    ret = dev->ll->uart_send(command, sizeof(command));
    ret |= dev->ll->uart_recv(response, sizeof(response));

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

    distanse = (uint16_t)response[3] << 8 | response[2];
    strength = (uint16_t)response[5] << 8 | response[4];
    dev->temperature_c = (((uint16_t)response[7] << 8 | response[6]) / 8 - 256);

    return_dist->distance_cm = distanse;
    return_dist->err_reason = TFMINIS_DATA_IS_VALID;
    return_dist->probability = (strength / TF_STRENGHT_PROB_CONVERTION_RANGE_COEFF) + 1;

    /* distance more than max operating range */
    if (distanse == TF_DIST_TARGET_TOO_FAR && strength < TF_STRENGHT_TARGET_TOO_FAR) {
        return_dist->probability = 0;
        return_dist->err_reason = TFMINIS_TOO_FAR_TARGET;
        return TFMINIS_FAIL;
    }

    /* distance less than min operating range */
    if (distanse == TF_DIST_TARGET_TOO_CLOSE && strength == TF_STRENGHT_TARGET_TOO_CLOSE) {
        return_dist->probability = 0;
        return_dist->err_reason = TFMINIS_TOO_CLOSE_TARGET;
        return TFMINIS_FAIL;
    }

    /* too much ambient light */
    if (distanse == TF_DIST_AMBIENT_SATURATION) {
        return_dist->probability = 0;
        return_dist->err_reason = TFMINIS_TOO_CLOSE_TARGET;
        return TFMINIS_FAIL;
    }

    return TFMINIS_OK;
}
