#ifndef __TF_MINI_S_H
#define __TF_MINI_S_H

/*
 * BEWARE !!!
 *
 * Conditions with Potential Malfunction:
 * 
 * 1. Detecting object with high reflectivity, such as the mirror or the smooth floor tile, may cause a system malfunction.
 * 2. The product will malfunction if there is any transparent object between it and the detecting object, such as glass or water.
 * 3. The product will be subject to risk of failure if its transmitting or receiving len is covered by the dust. Please keep the lens clean.
 * 4. Please do not directly touch circuit board of the product by hand as it is exposed.
 *    Please wear antistatic wrist strap or glove if necessary; Otherwise, the product will be subject to risk of failure,
 *    which is shown by failure of normal operation, and even it will be broken.
 **/

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

typedef enum {
    TFMINIS_OK,
    TFMINIS_FAIL
} tfminis_ret_t;

typedef enum {
    TFMINIS_USE_UART,
    TFMINIS_USE_I2C /* NOT SUPPORTED FOR NOW */
} tfminis_interfaces_t;

typedef enum {
    TFMINIS_TOO_FAR_TARGET, /* distance more than max operating range */
    TFMINIS_TOO_CLOSE_TARGET, /* distance less than min operating range */
    TFMINIS_AMBIENT_LIGHT_SATURATION,
    TFMINIS_CRC_FAILED,
} tfminis_dist_error_reason_t;

typedef struct {
    /* Millisecond software delay */
    void (*delay_ms)(uint32_t ms);

    /*
     * Send data in polling mode.
     * Must return 1 in case of success.
     * 
     * This function is used by driver ONLY to send commands frame to TFmini-S
     **/
    int (*uart_send)(uint8_t *data, size_t len);

    /*
     * Receive data in polling mode.
     * Must return 1 in case of success.
     * 
     * This function is used by driver ONLY to receive responce for commands.
     * For data frame receiving setup DMA or interrupt for your chip on call
     * 
     **/
    int (*uart_recv)(uint8_t *data, size_t len);

    uint32_t (*crc32)(uint8_t *data, size_t len);
} tfminis_ll_t;

typedef struct {
    /* measured distance to object in cm */
    uint16_t distance_cm;

    /* accuracy of the dist; ±6cm for 0.1m < range < 6m; 1% for 6m-12m */
    uint8_t accuracy_cm;

    /* 
     * Probability that the data is correct (0-100%).
     * It depends on laser received signal strength, frame range,
     * target reflectivity, temperature etc. 
     */
    uint8_t probability;

    tfminis_dist_error_reason_t err_reason;
} tfminis_dist_t;

typedef struct {
    tfminis_ll_t *ll; /* Low Level functions which must be implemented by user */
    tfminis_interfaces_t interface;
    tfminis_dist_t dist;
    uint16_t temperature;
} tfminis_dev_t;

/* Fill in dev->ll with appropriate functions before call */
tfminis_ret_t tfminis_init(tfminis_dev_t *dev, tfminis_interfaces_t interf);

/* The function will put sensor's limits to the passed pointers. Values will be in cm */
tfminis_ret_t tfminis_get_operating_range(tfminis_dev_t *dev, uint16_t *min_cm, uint16_t *max_cm);

/*
 * Obtain distance data
 * If returns TFMINIS_FAIL, check the reason with the help of tfminis_get_err_reason() 
 */
tfminis_dist_t tfminis_get_distance(tfminis_dev_t *dev);
tfminis_dist_error_reason_t tfminis_get_err_reason(tfminis_dev_t *dev);

/* Returns last successfully obtained TFmini-S chip temperature */
uint16_t tfminis_get_chip_temp(tfminis_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif  // __TF_MINI_S_H
