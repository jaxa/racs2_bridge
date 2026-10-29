/* SPDX-License-Identifier: Apache-2.0 */
#ifndef RACS2_USER_MSG_H
#define RACS2_USER_MSG_H
#include "cfe.h"
#include "racs2_bridge_wire.h"
/* Retain the telemetry-sized SB header storage of the original RACS2
 * envelope for both MID types. CFE_MSG_Init sets its actual header type. */
typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    char ros2_topic_name[RACS2_HEADER_SIZE];
    uint8 body_data_length;
    uint8 body_data[RACS2_BODY_MAX];
} racs2_user_msg_t;
#define RACS2_USER_MSG_LNGTH sizeof(racs2_user_msg_t)
#define ROS2_TOPIC_NAME_LNGTH RACS2_HEADER_SIZE
#define BODY_DATA_MAX_LNGTH RACS2_BODY_MAX
#endif
