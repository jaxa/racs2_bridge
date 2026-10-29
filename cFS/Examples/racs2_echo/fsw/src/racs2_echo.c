/* SPDX-License-Identifier: Apache-2.0 */
/* Optional raw-binary example, not part of a normal flight configuration. */
#include "racs2_user_msg.h"
#include "racs2_bridge_client_msgids.h"
#include <string.h>
#ifndef RACS2_ECHO_MID
#define RACS2_ECHO_MID 0x1894
#endif
void RACS2_ECHO_Main(void)
{
    CFE_SB_PipeId_t pipe;
    uint32 run_status = CFE_ES_RunStatus_APP_RUN;
    CFE_Status_t status = CFE_SB_CreatePipe(&pipe, 16, "RACS2_ECHO");
    if (status == CFE_SUCCESS)
        status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(RACS2_ECHO_MID), pipe);
    if (status != CFE_SUCCESS)
        run_status = CFE_ES_RunStatus_APP_ERROR;
    while (CFE_ES_RunLoop(&run_status))
    {
        CFE_SB_Buffer_t *buffer;
        const racs2_user_msg_t *input;
        racs2_user_msg_t output;
        CFE_MSG_Size_t size;
        status = CFE_SB_ReceiveBuffer(&buffer, pipe, 1000);
        if (status != CFE_SUCCESS)
            continue;
        input = (const racs2_user_msg_t *)buffer;
        if (CFE_MSG_GetSize(&buffer->Msg, &size) != CFE_SUCCESS ||
            size < offsetof(racs2_user_msg_t, body_data) || input->body_data_length > RACS2_BODY_MAX ||
            size < offsetof(racs2_user_msg_t, body_data) + input->body_data_length)
            continue;
        memset(&output, 0, sizeof(output));
        CFE_MSG_Init(CFE_MSG_PTR(output.TelemetryHeader),
                      CFE_SB_ValueToMsgId(RACS2_BRIDGE_MID), sizeof(output));
        strcpy(output.ros2_topic_name, "/Recv/RACS2Bridge");
        output.body_data_length = input->body_data_length;
        memcpy(output.body_data, input->body_data, input->body_data_length);
        CFE_SB_TransmitMsg(CFE_MSG_PTR(output.TelemetryHeader), true);
    }
    CFE_ES_ExitApp(run_status);
}
