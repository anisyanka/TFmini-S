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

#define TFMINIS_OPERATING_RANGE_MIN_CM (10U)
#define TFMINIS_OPERATING_RANGE_MAX_CM (1200U)

#if (TFMINIS_DEBUG_MODE_LOGGING == 1)
# define tfminis_debug_logger_func(...) "place your logger function here; For instance: printf(__VA_ARGS__)"
#endif

typedef enum {
    TFMINIS_OK,
    TFMINIS_FAIL,

    /* spectial error codes */
    TFMINIS_INTERF_ERR,
    TFMINIS_WRONG_ARGS,
    TFMINIS_WRONG_RESPONSE,
    TFMINIS_WRONG_CRC,
    TFMINIS_FAIL_SW_RESET,
} tfminis_ret_t;

typedef enum {
    TFMINIS_USE_UART,
    TFMINIS_USE_I2C /* NOT SUPPORTED FOR NOW */
} tfminis_interfaces_t;

typedef enum {
    TFMINIS_DATA_IS_VALID, /* everything is alright */
    TFMINIS_TOO_FAR_TARGET, /* distance more than max operating range */
    TFMINIS_TOO_CLOSE_TARGET, /* distance less than min operating range */
    TFMINIS_AMBIENT_LIGHT_SATURATION,
    TFMINIS_CRC_FAILED,
} tfminis_dist_error_reason_t;

typedef struct {
    /* Millisecond software delay */
    void (*delay_ms)(uint32_t ms);

    /* Send data in POLLING mode.
     * Must return 0 in case of success.
     * 
     * This function is used by driver ONLY to send commands frame to TFmini-S.
     * Obtatinig data frames has to stop to send/recv cmd data to/from TFmini-S **/
    int (*uart_send)(uint8_t *data, size_t len);

    /* Receive data in POLLING mode.
     * Must return 0 in case of success.
     * 
     * This function is used by driver ONLY to receive responce for commands.
     * For data frame receiving setup DMA or uart interrupt for your chip **/
    int (*uart_recv)(uint8_t *data, size_t len);

    /* Enable/disable whatever you use to asynchronously receive data. DMA or UART IRQ.
     * It is needed to setup TFmidi-S with some commands in polling mode.
     Must return 0 in case of success. */
    int (*start_dma_or_irq_operations)(void);
    int (*stop_dma_or_irq_operations)(void);
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
    uint32_t fw_version;
    tfminis_ll_t *ll; /* Low Level functions which must be implemented by user */
    tfminis_interfaces_t interface_type;
    tfminis_dist_t dist;
    uint16_t temperature_c; /* Degree Celsius */
} tfminis_dev_t;

/* Fill in dev->ll with appropriate functions before call.
 * Default setup is:
 *  - out enable
 *  - 115200 baud
 *  - format is standard 9 bytes(cm)
 *  - 100Hz output frame
 *  - strength Threshold = 100
 * FW version of TFmini-S is placed to dev->fw_version
 **/
tfminis_ret_t tfminis_init(tfminis_dev_t *dev, tfminis_interfaces_t interf);

/* Obtain distance data.
 * If returns TFMINIS_FAIL, check the reason with the help of tfminis_get_err_reason() */
tfminis_dist_t tfminis_get_distance(tfminis_dev_t *dev);
tfminis_dist_error_reason_t tfminis_get_err_reason(tfminis_dev_t *dev);

/* Returns last obtained TFmini-S chip temperature.  */
uint16_t tfminis_get_chip_temp(tfminis_dev_t *dev);

/* How often you want to obtain frames? (0 - 1000 Hz).
 * Remember the higher frame rate the higher interface speed required. (baudrate in case of UART)
 *
 * 0Hz means that you have to manually use tfminis_trigger_detection() func by itselt to obtain distanse.
 * You must disable async data receiving (DMA or UART interrupts), because TFmini-S for now 
 * will send data onlu after triggering */
tfminis_ret_t tfminis_set_frame_rate(tfminis_dev_t *dev);
tfminis_ret_t tfminis_trigger_detection(tfminis_dev_t *dev, tfminis_dist_t *return_dist);

#ifdef __cplusplus
}
#endif

#endif  /* __TF_MINI_S_H */
