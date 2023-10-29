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

/* Count of TFmini-S sensor in your setup. Used to parse data from several devices */
#ifndef TFMINIS_DEVICES_IN_SYSTEM
 #define TFMINIS_DEVICES_IN_SYSTEM 1
#endif

typedef enum {
    TFMINIS_OK,
    TFMINIS_FAIL,
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
    TFMINIS_STRENGTH_SATURATION, /* too good reflectivity */
    TFMINIS_AMBIENT_LIGHT_SATURATION, /* too much ambient light */
    TFMINIS_CRC_FAILED,
    TFMINIS_NOT_A_DATA_FRAME, /* start of frame not equal 0x59 */
    TFMINIS_INTF_ERR, /* error during uart send/recv functions */
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
     * Must return 0 in case of success. If you don't use irq\dma just make stub function */
    int (*start_dma_or_irq_operations)(void);
    int (*stop_dma_or_irq_operations)(void);

    /* When data frame has been received via UART IRQ or DMA idle event IRQ
     * this callback will be called. If not used set it to NULL.
     * Remember that it will be called in uart byte received interrupt (in case of using UART IRQ)
     * or in DMA idle event received interrupt (in case of using DMA IDLE interrupt).
     * It is supposed to be used to immediately put new data to some RTOS queue to wake up waiting RTOS task */
    void (*data_avaliabe_isr_cb)(void);
} tfminis_ll_t;

typedef struct {
    uint16_t distance_cm;
    uint16_t strength;
    tfminis_dist_error_reason_t err_reason;
} tfminis_dist_t;

typedef struct {
    uint32_t fw_version;
    tfminis_ll_t *ll; /* Low Level functions which must be implemented by user */
    tfminis_interfaces_t interface_type;
    tfminis_dist_t dist; /* last measured data when IRQ or DMA used */
    uint16_t temperature_c; /* Degree Celsius */

    /* Private variables. Don't use it directly! */
    void *_private;
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
 * 0Hz means that you have to manually use tfminis_get_distance_oneshot() func by itselt to obtain distanse.
 * You must disable async data receiving (DMA or UART interrupts), because TFmini-S for now 
 * will send data onlu after triggering */
tfminis_ret_t tfminis_set_frame_rate(tfminis_dev_t *dev, uint16_t frame_rate);

/* Don't use it if frame rate != and if UART IRQ or DMA are used.
 * If ret value = TFMINIS_FAIL, check the reason via return_dist.err_reason */
tfminis_ret_t tfminis_get_distance_oneshot(tfminis_dev_t *dev, tfminis_dist_t *return_dist);

/* If you are going use UART interrupts to receive data, call this func in ISR.
 * But is better to use DMA+idle event interrupt to reduce uart interrupt overhead.
 * In case of using uart interrupt don't set higt frame rate.
 * 
 * The function will parse bytes automatically and place it in dev.
 * The last measurement will be available with the help of tfminis_get_distance() API. 
 * 
 * Don't use tfminis_get_distance_oneshot() in case of IRQ */
void tfminis_handle_rx_byte_uart_isr(tfminis_dev_t *dev, uint8_t byte);

/* Function to call in DMA idle event interrupt
 * The function will parse data automatically and place all in dev.
 * The last measurement will be available with the help of tfminis_get_distance() API.
 * 
 * Don't use tfminis_get_distance_oneshot() in case of DMA IRQ */
void tfminis_handle_rx_data_dma_isr(tfminis_dev_t *dev, uint8_t *rx_frame, size_t len);

#ifdef __cplusplus
}
#endif

#endif  /* __TF_MINI_S_H */
