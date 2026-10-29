/* SPDX-License-Identifier: Apache-2.0 */
#include "cfe.h"
#include "racs2_bridge_client_msgids.h"
#include "racs2_user_msg.h"
#include <libwebsockets.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define RACS2_QUEUE_DEPTH 32u
typedef struct
{
    unsigned char bytes[LWS_PRE + RACS2_MESSAGE_MAX];
    size_t size;
} RACS2_Frame;
static struct
{
    CFE_SB_PipeId_t pipe;
    struct lws_context *context;
    struct lws *connection;
    lws_sorted_usec_list_t wakeup;
    int connected;
    char host[256];
    int port;
    time_t retry_at;
    uint8_t rx[RACS2_MESSAGE_MAX];
    size_t rx_size;
    RACS2_Frame tx[RACS2_QUEUE_DEPTH];
    unsigned int head;
    unsigned int count;
} Bridge;

/* cFE calls and websocket callbacks run on the same cFS application task. */
static void RACS2_Receive(const uint8_t *bytes, size_t size)
{
    racs2_user_msg_t packet;
    CFE_MSG_Type_t type;
    CFE_Status_t status;
    CFE_SB_MsgId_t mid;
    if (!racs2_wire_valid_size(size))
        return;
    memset(&packet, 0, sizeof(packet));
    mid = CFE_SB_ValueToMsgId(racs2_wire_mid(bytes));
    status = CFE_MSG_Init(CFE_MSG_PTR(packet.TelemetryHeader), mid, sizeof(packet));
    if (status != CFE_SUCCESS)
        return;
    packet.body_data_length = (uint8)(size - RACS2_HEADER_SIZE);
    memcpy(packet.body_data, bytes + RACS2_HEADER_SIZE, packet.body_data_length);
    if (CFE_MSG_GetType(CFE_MSG_PTR(packet.TelemetryHeader), &type) == CFE_SUCCESS &&
        type == CFE_MSG_Type_Tlm)
        CFE_SB_TimeStampMsg(CFE_MSG_PTR(packet.TelemetryHeader));
    status = CFE_SB_TransmitMsg(CFE_MSG_PTR(packet.TelemetryHeader), true);
    if (status != CFE_SUCCESS)
        CFE_ES_WriteToSysLog("RACS2: SB transmit failed: %ld\n", (long)status);
}

static int RACS2_Callback(struct lws *wsi, enum lws_callback_reasons reason,
                         void *user, void *in, size_t len)
{
    (void)user;
    switch (reason)
    {
        case LWS_CALLBACK_CLIENT_ESTABLISHED:
            Bridge.connection = wsi;
            Bridge.connected = 1;
            Bridge.rx_size = 0;
            break;
        case LWS_CALLBACK_CLIENT_RECEIVE:
            if (lws_is_first_fragment(wsi))
                Bridge.rx_size = 0;
            if (!lws_frame_is_binary(wsi) || len > sizeof(Bridge.rx) - Bridge.rx_size)
                return -1;
            memcpy(Bridge.rx + Bridge.rx_size, in, len);
            Bridge.rx_size += len;
            if (lws_is_final_fragment(wsi) && lws_remaining_packet_payload(wsi) == 0)
            {
                RACS2_Receive(Bridge.rx, Bridge.rx_size);
                Bridge.rx_size = 0;
            }
            break;
        case LWS_CALLBACK_CLIENT_WRITEABLE:
            if (Bridge.count != 0)
            {
                RACS2_Frame *frame = &Bridge.tx[Bridge.head];
                if (lws_write(wsi, frame->bytes + LWS_PRE, frame->size, LWS_WRITE_BINARY) < (int)frame->size)
                    return -1;
                Bridge.head = (Bridge.head + 1) % RACS2_QUEUE_DEPTH;
                --Bridge.count;
                if (Bridge.count != 0)
                    lws_callback_on_writable(wsi);
            }
            break;
        case LWS_CALLBACK_CLIENT_CLOSED:
        case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
            Bridge.connection = NULL;
            Bridge.connected = 0;
            Bridge.count = 0;
            Bridge.rx_size = 0;
            Bridge.retry_at = time(NULL) + 1;
            break;
        default:
            break;
    }
    return 0;
}
static const struct lws_protocols RACS2_Protocols[] = {
    {.name = "example-protocol", .callback = RACS2_Callback,
     .per_session_data_size = 0, .rx_buffer_size = RACS2_MESSAGE_MAX}, {0}};

/* Modern lws_service ignores its timeout argument. A scheduled callback
 * wakes this task to check its SB pipe every 10 ms without busy polling. */
static void RACS2_Wakeup(lws_sorted_usec_list_t *timer)
{
    lws_sul_schedule(Bridge.context, 0, timer, RACS2_Wakeup, 10000);
}
static int RACS2_ReadConfig(void)
{
    char line[512];
    const char *path = getenv("RACS2_CONFIG");
    FILE *file = fopen(path ? path : "cf/racs2_bridge_config.txt", "r");
    if (file == NULL)
        return -1;
    while (fgets(line, sizeof(line), file))
    {
        char *key = strtok(line, "= \t\r\n");
        char *value = strtok(NULL, " \t\r\n");
        if (!key || !value || key[0] == '#')
            continue;
        if (strcmp(key, "wss_uri") == 0)
        {
            if (strlen(value) >= sizeof(Bridge.host))
            {
                fclose(file);
                return -1;
            }
            strcpy(Bridge.host, value);
        }
        else if (strcmp(key, "wss_port") == 0)
        {
            char *end;
            long port;
            errno = 0;
            port = strtol(value, &end, 10);
            if (errno || *end || port < 1 || port > 65535)
            {
                fclose(file);
                return -1;
            }
            Bridge.port = (int)port;
        }
    }
    fclose(file);
    return Bridge.host[0] && Bridge.port ? 0 : -1;
}
static CFE_Status_t RACS2_Init(void)
{
    struct lws_context_creation_info info;
    CFE_Status_t status;
    memset(&Bridge, 0, sizeof(Bridge));
    if (RACS2_ReadConfig() != 0)
    {
        CFE_ES_WriteToSysLog("RACS2: cannot read valid bridge configuration\n");
        return CFE_SB_BAD_ARGUMENT;
    }
    CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    status = CFE_SB_CreatePipe(&Bridge.pipe, RACS2_QUEUE_DEPTH, "RACS2_TO_ROS");
    if (status != CFE_SUCCESS)
        return status;
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(RACS2_BRIDGE_MID), Bridge.pipe);
    if (status != CFE_SUCCESS)
        return status;
    memset(&info, 0, sizeof(info));
    info.port = CONTEXT_PORT_NO_LISTEN;
    info.protocols = RACS2_Protocols;
    info.gid = -1;
    info.uid = -1;
    Bridge.context = lws_create_context(&info);
    if (Bridge.context == NULL)
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    lws_sul_schedule(Bridge.context, 0, &Bridge.wakeup, RACS2_Wakeup, 10000);
    CFE_EVS_SendEvent(41, CFE_EVS_EventType_INFORMATION, "RACS2 cFE 7 client initialized");
    return CFE_SUCCESS;
}
static void RACS2_QueueSB(const CFE_SB_Buffer_t *buffer)
{
    const racs2_user_msg_t *packet = (const racs2_user_msg_t *)buffer;
    CFE_MSG_Size_t size;
    RACS2_Frame *frame;
    if (!Bridge.connected || Bridge.count == RACS2_QUEUE_DEPTH)
        return;
    if (CFE_MSG_GetSize(&buffer->Msg, &size) != CFE_SUCCESS ||
        size < offsetof(racs2_user_msg_t, body_data) || packet->body_data_length > RACS2_BODY_MAX ||
        size < offsetof(racs2_user_msg_t, body_data) + packet->body_data_length)
        return;
    frame = &Bridge.tx[(Bridge.head + Bridge.count) % RACS2_QUEUE_DEPTH];
    memcpy(frame->bytes + LWS_PRE, packet->ros2_topic_name, RACS2_HEADER_SIZE);
    memcpy(frame->bytes + LWS_PRE + RACS2_HEADER_SIZE, packet->body_data, packet->body_data_length);
    frame->size = RACS2_HEADER_SIZE + packet->body_data_length;
    ++Bridge.count;
    lws_callback_on_writable(Bridge.connection);
}
void RACS2_CLIENT_Main(void)
{
    uint32 run_status = CFE_ES_RunStatus_APP_RUN;
    if (RACS2_Init() != CFE_SUCCESS)
        run_status = CFE_ES_RunStatus_APP_ERROR;
    while (CFE_ES_RunLoop(&run_status))
    {
        unsigned int count;
        if (Bridge.connection == NULL && time(NULL) >= Bridge.retry_at)
        {
            struct lws_client_connect_info info;
            memset(&info, 0, sizeof(info));
            info.context = Bridge.context;
            info.address = Bridge.host;
            info.port = Bridge.port;
            info.path = "/";
            info.host = Bridge.host;
            info.protocol = RACS2_Protocols[0].name;
            Bridge.retry_at = time(NULL) + 1;
            lws_client_connect_via_info(&info);
        }
        for (count = 0; count < RACS2_QUEUE_DEPTH; ++count)
        {
            CFE_SB_Buffer_t *buffer;
            if (CFE_SB_ReceiveBuffer(&buffer, Bridge.pipe, CFE_SB_POLL) != CFE_SUCCESS)
                break;
            RACS2_QueueSB(buffer);
        }
        if (lws_service(Bridge.context, 0) < 0)
            run_status = CFE_ES_RunStatus_APP_ERROR;
    }
    if (Bridge.context)
    {
        lws_sul_schedule(Bridge.context, 0, &Bridge.wakeup,
                         RACS2_Wakeup, LWS_SET_TIMER_USEC_CANCEL);
        lws_context_destroy(Bridge.context);
    }
    CFE_ES_ExitApp(run_status);
}
