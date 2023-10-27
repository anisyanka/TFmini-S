#include "tf_mini_s.h"

#define TF_OPERATING_RANGE_MIN_CM 10U
#define TF_OPERATING_RANGE_MAX_CM 1200U

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
