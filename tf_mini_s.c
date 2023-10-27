#include "tf_mini_s.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define TF_FRAME_SIZE (9U)
#define TF_FRAME_PAYLOAD_SIZE (6U)
#define TF_FRAME_START_BYTE (0x59)

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

static tfminis_ret_t system_reset(tfminis_dev_t *dev)
{
    int ret = 0;
    uint8_t command[4] = { 0x5A, 0x04, 0x02, 0x60 };
    uint8_t response[5] = { 0 };

    ret = dev->ll->uart_send(command, sizeof(command));
    ret |= dev->ll->uart_recv(response, sizeof(response));

    if (ret) {
        return TFMINIS_INTERF_ERR;
    }

    if (response[0] != 0x5A || response[1] != 0x05 || response[2] != 0x02) {
        return TFMINIS_WRONG_RESPONSE;
    }

    if (response[3] != 0x00 || response[4] != 0x60) {
        return TFMINIS_FAIL_SW_RESET;
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

    /* Stop data obtaining to use polling to setup the sensor */
    if (dev->ll->stop_dma_or_irq_operations()) {
        return TFMINIS_INTERF_ERR;
    }
    dev->ll->delay_ms(10);

    ret = system_reset(dev);
    if (ret != TFMINIS_OK) {
        return ret;
    }
    dev->ll->delay_ms(1000);

    /* Disable data output. Call command one more times to prevent previous data frame reading */
    {
        uint8_t command[] = { 0x5A, 0x05, 0x07, 0x00, 0x66 };
        uint8_t response[sizeof(command)] = { 0 };

        ret = dev->ll->uart_send(command, sizeof(command));
        ret |= dev->ll->uart_recv(response, sizeof(response));
        dev->ll->delay_ms(20);
        ret |= dev->ll->uart_send(command, sizeof(command));
        ret |= dev->ll->uart_recv(response, sizeof(response));
        if (ret) {
            return TFMINIS_INTERF_ERR;
        }

        if (memcmp(command, response, sizeof(command)) != 0) {
            return TFMINIS_WRONG_RESPONSE;
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
#if (TFMINIS_DEBUG_MODE_LOGGING == 1)
    tfminis_debug_logger_func("[TFmini-S] FW version is V%d.%d.%d\n", response[5], response[4], response[3]);
#endif
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

tfminis_ret_t tfminis_trigger_detection(tfminis_dev_t *dev, tfminis_dist_t *return_dist)
{
    return TFMINIS_OK;
}
