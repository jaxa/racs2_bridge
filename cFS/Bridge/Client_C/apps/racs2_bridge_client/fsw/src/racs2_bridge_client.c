/*
**   Include Files:
*/

#include "racs2_bridge_client.h"
#include "racs2_bridge_client_perfids.h"
#include "racs2_bridge_client_msgids.h"
#include "racs2_bridge_client_msg.h"
#include "racs2_user_msg.h"
#include "racs2_bridge_msg.h"
#include "racs2_bridge_client_events.h"
#include "racs2_bridge_client_version.h"
#include <pthread.h>

/*
** global data
*/

sample_hk_tlm_t    RACS2_BRIDGE_CLIENT_HkTelemetryPkt;
sample_hk_tlm_t    SAMPLE_TalkerListenerPkt;
CFE_SB_PipeId_t    RACS2_BRIDGE_CLIENT_CommandPipe;
CFE_SB_MsgPtr_t    RACS2_BRIDGE_CLIENT_MsgPtr;
racs2_user_msg_t   RACS2_UserMsgPkt;
CFE_SB_MsgPtr_t    RACS2_UserMsgPkt_Ptr;
char     wss_uri[16];
uint16_t wss_port;

static CFE_EVS_BinFilter_t  SAMPLE_EventFilters[] =
       {  /* Event ID    mask */
          {SAMPLE_STARTUP_INF_EID,       0x0000},
          {SAMPLE_COMMAND_ERR_EID,       0x0000},
          {SAMPLE_COMMANDNOP_INF_EID,    0x0000},
          {SAMPLE_COMMANDRST_INF_EID,    0x0000},
       };

pthread_mutex_t g_bridge_msg_pkt_mutex;
uint8_t g_should_send_msg_pkt = 0;
uint8_t g_bridge_msg_pkt[BRIDGE_HEADER_LNGTH+BODY_DATA_MAX_LNGTH];

/* -- websocket settings -------- */
#include <libwebsockets.h>

static struct lws *web_socket = NULL;
static struct lws* g_wsi_ptr = NULL;
enum protocols
{
    PROTOCOL_EXAMPLE = 0,
    PROTOCOL_COUNT
};

#define EXAMPLE_RX_BUFFER_BYTES (256)
#define RACS2_BRIDGE_HEADER_LENGTH 32
#define RACS2_BRIDGE_DEST_MSGID_NUM 16
uint8_t registerd_msgid_num = 0;
uint16 dest_message_id_list[RACS2_BRIDGE_DEST_MSGID_NUM] = {0};

static int callback_example( struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len )
{
    lwsl_user( "BBB: callback_example()\n" ) ;
    g_wsi_ptr = wsi;

    switch( reason )
    {
        case LWS_CALLBACK_CLIENT_ESTABLISHED:
            lwsl_user( "case LWS_CALLBACK_CLIENT_ESTABLISHED: \n" ) ;
            lws_callback_on_writable( wsi );
            break;

        case LWS_CALLBACK_CLIENT_RECEIVE:
            lwsl_user( "case LWS_CALLBACK_CLIENT_RECEIVE: \n" ) ;
            lwsl_user( "[Recv]: %s\n", (char*)in ) ;
            lwsl_user( "[Recv]: data len = %ld\n", len ) ;

            // === send message =========================
            char *hello = "Hello";
            if (!strncmp((char*)in, hello, 1))
            {
                // OS_printf("[Recv]: %s\n", (char*)in);
                break;
            }
            // Get message ID from header
            uint16_t id_seg1 = ((uint8_t*)in)[0];
            uint16_t id_seg2 = ((uint8_t*)in)[1];
            uint16_t message_id = id_seg1 << 8 | id_seg2;
            OS_printf("RACS2_BRIDGE_CLIENT: dest cFS message ID : 0x%x\n", message_id);
            // if (is_new_msgid(message_id)) {
            //     // CFE_SB_InitMsg(&RACS2_UserMsgPkt, RACS2_BRIDGE_MID, RACS2_USER_MSG_LNGTH, false);
            //     CFE_SB_InitMsg(&RACS2_UserMsgPkt, message_id, RACS2_USER_MSG_LNGTH, true);
            //     OS_printf("RACS2_BRIDGE_CLIENT: CFE_SB_InitMsg for MsgId[%x]\n\n\n\n\n\n\n", message_id);
            // }
            CFE_SB_InitMsg(&RACS2_UserMsgPkt, message_id, RACS2_USER_MSG_LNGTH, true);
            // Set body data length
            uint8_t body_data_length = len - RACS2_BRIDGE_HEADER_LENGTH;
            RACS2_UserMsgPkt.body_data_length = body_data_length;
            OS_printf("RACS2_BRIDGE_CLIENT: body data length : %d\n", body_data_length);
            // Copy body data
            memcpy(RACS2_UserMsgPkt.body_data, (uint8_t*)in + RACS2_BRIDGE_HEADER_LENGTH, body_data_length);
            // Send message
            CFE_SB_TimeStampMsg((CFE_SB_Msg_t *) &RACS2_UserMsgPkt);
            int32 status = CFE_SB_SendMsg((CFE_SB_Msg_t *) &RACS2_UserMsgPkt);
            // OS_printf("RACS2_BRIDGE_CLIENT: Sent message, MID = [0x%x], sample_command_count = %d\n",
            //     CFE_SB_GetMsgId((CFE_SB_MsgPtr_t) &RACS2_UserMsgPkt),
            //     RACS2_UserMsgPkt.sample_command_count
            //     );
            OS_printf("RACS2_BRIDGE_CLIENT: Sent message, MID = [0x%x]\n",
                CFE_SB_GetMsgId((CFE_SB_MsgPtr_t) &RACS2_UserMsgPkt)
            );
            if (status != CFE_SUCCESS) {
                OS_printf("RACS2_BRIDGE_CLIENT: Error: sending is failed. \n");
            }


            break;

        case LWS_CALLBACK_CLIENT_WRITEABLE:
        {
            lwsl_user( "case LWS_CALLBACK_CLIENT_WRITEABLE: \n" ) ;

            pthread_mutex_lock(&g_bridge_msg_pkt_mutex);
            if (g_should_send_msg_pkt)
            {
              lws_write( wsi, g_bridge_msg_pkt, BRIDGE_HEADER_LNGTH+BODY_DATA_MAX_LNGTH, LWS_WRITE_BINARY );
              g_should_send_msg_pkt = 0;
            }
            pthread_mutex_unlock(&g_bridge_msg_pkt_mutex);
            break;
        }

        case LWS_CALLBACK_CLOSED:
        case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
            lwsl_user( "case LWS_CALLBACK_CLOSED or LWS_CALLBACK_CLIENT_CONNECTION_ERROR: \n" ) ;
            web_socket = NULL;
            break;

        default:
            lwsl_user( "case : default\n" ) ;
            break;
    }

    return 0;
}

static struct lws_protocols protocols[] =
{
    {
        "example-protocol",
        callback_example,
        0,
        EXAMPLE_RX_BUFFER_BYTES,
    },
    { NULL, NULL, 0, 0 } /* terminator */
};

struct lws_context_creation_info g_info;
struct lws_context * g_context ;
time_t g_old = 0;

/** * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* RACS2_BRIDGE_CLIENT_Main() -- Application entry point and main process loop          */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * * * * * * **/
void RACS2_BRIDGE_CLIENT_Main( void )
{
    int32  status;
    uint32 RunStatus = CFE_ES_RunStatus_APP_RUN;

    OS_printf("RACS2_BRIDGE_CLIENT_Main starts.\n");

    CFE_ES_PerfLogEntry(SAMPLE_APP_PERF_ID);

    RACS2_BRIDGE_CLIENT_Init();

    // == For WebScoket =======================
    memset( &g_info, 0, sizeof( g_info ) );
    g_info.port      = CONTEXT_PORT_NO_LISTEN;
    g_info.protocols = protocols;
    g_info.gid       = -1;
    g_info.uid       = -1;

    g_context = lws_create_context( &g_info );

    lws_set_log_level( LLL_USER, NULL);
    lwsl_user( "[BBB]: client - main\n" );
    // ========================================

    /*
    ** RACS2_BRIDGE_CLIENT Runloop
    */
    while (CFE_ES_RunLoop(&RunStatus) == true)
    {
        CFE_ES_PerfLogExit(SAMPLE_APP_PERF_ID);

        // === BBB: For WebSocket ===============
        struct timeval tv;
        gettimeofday( &tv, NULL );

        /* Connect if we are not connected to the server. */
        if( ! web_socket && tv.tv_sec != g_old )
        {
            struct lws_client_connect_info ccinfo = {0};
            ccinfo.context = g_context ;
            ccinfo.address = wss_uri;
            ccinfo.port = wss_port;
            ccinfo.path = "/";
            ccinfo.host = lws_canonical_hostname( g_context );
            ccinfo.origin = "origin";
            ccinfo.protocol = protocols[PROTOCOL_EXAMPLE].name;
            web_socket = lws_client_connect_via_info(&ccinfo);
        }

        if( tv.tv_sec != g_old )
        {
            /* Send a random number to the server every second. */
            lws_callback_on_writable( web_socket );
            g_old = tv.tv_sec;
        }

        /* Pend on receipt of command packet -- timeout set to 500 millisecs */
        status = CFE_SB_RcvMsg(&RACS2_UserMsgPkt_Ptr, RACS2_BRIDGE_CLIENT_CommandPipe, 1000);

        CFE_ES_PerfLogEntry(SAMPLE_APP_PERF_ID);

        if (status == CFE_SUCCESS)
        {
            RACS2_BRIDGE_CLIENT_ProcessCommandPacket();
        }
        else
        {
            OS_printf("RACS2_BRIDGE_CLIENT: CFE_SB_RcvMsg failed, status = 0x%x\n", status);
        }

        lws_service( g_context, (0) );
        // === EEE: For WebSocket =====================
    }

    CFE_ES_ExitApp(RunStatus);

} /* End of RACS2_BRIDGE_CLIENT_Main() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  */
/*                                                                            */
/* RACS2_BRIDGE_CLIENT_Init() --  initialization                              */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void RACS2_BRIDGE_CLIENT_Init(void)
{
    int32 status;

    /**
     * Read config
    */
    FILE *fp;
    char line[64];
    char *str;
    char key[16];
    char value[16];

    if ((fp = fopen("cf/racs2_bridge_config.txt", "r")) == NULL)
    {
        OS_printf("failed to open racs2_bridge_config.txt\n");
        exit(-1);
    }

    char *ch_wss_uri  = "wss_uri";
    char *ch_wss_port = "wss_port";
    while ((str = fgets(line, 128, fp)) != NULL) {
        sscanf(line, "%[^=]=%s", key, value);
        // read uri
        if (strcmp(key, ch_wss_uri) == 0)
        {
            strcpy(wss_uri, value);
            OS_printf("wss_uri = [%s]\n", wss_uri);
        }
        // read port
        if (strcmp(key, ch_wss_port) == 0)
        {
            wss_port = atoi(value);
            OS_printf("wss_port = [%d]\n", wss_port);
        }
    }

    fclose(fp);

    /*
    ** Register the app with Executive services
    */
    CFE_ES_RegisterApp() ;

    /*
    ** Register the events
    */
    CFE_EVS_Register(SAMPLE_EventFilters,
                     sizeof(SAMPLE_EventFilters)/sizeof(CFE_EVS_BinFilter_t),
                     CFE_EVS_EventFilter_BINARY);

    /*
    ** Create the Software Bus command pipe and subscribe to housekeeping
    **  messages
    */
    status = CFE_SB_CreatePipe(&RACS2_BRIDGE_CLIENT_CommandPipe, SAMPLE_PIPE_DEPTH, "TO_ROS");
    if (status != CFE_SUCCESS)
    {
        OS_printf("RACS2_BRIDGE_CLIENT: failed to create SB pipe, status = 0x%x\n", status);
    }
    status = CFE_SB_Subscribe(RACS2_BRIDGE_MID, RACS2_BRIDGE_CLIENT_CommandPipe);
    if (status != CFE_SUCCESS)
    {
        OS_printf("RACS2_BRIDGE_CLIENT: failed to subscribe MsgID[0x%x], status = 0x%x\n", RACS2_BRIDGE_MID, status);
    }
    RACS2_BRIDGE_CLIENT_ResetCounters();

    CFE_SB_InitMsg(&RACS2_BRIDGE_CLIENT_HkTelemetryPkt,
                   RACS2_BRIDGE_CLIENT_HK_TLM_MID,
                   SAMPLE_APP_HK_TLM_LNGTH, true);

    CFE_EVS_SendEvent (SAMPLE_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION,
               "RACS2_BRIDGE_CLIENT App Initialized. Version %d.%d.%d.%d",
                SAMPLE_APP_MAJOR_VERSION,
                SAMPLE_APP_MINOR_VERSION,
                SAMPLE_APP_REVISION,
                SAMPLE_APP_MISSION_REV);

} /* End of RACS2_BRIDGE_CLIENT_Init() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * ***/
/*  Name:  RACS2_BRIDGE_CLIENT_ProcessCommandPacket                                            */
/*                                                                                         */
/*  Purpose:                                                                               */
/*     This routine will process any packet that is received on the RACS2_BRIDGE_CLIENT    */
/*     command pipe.                                                                       */
/*                                                                                         */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * * * * * * * **/
void RACS2_BRIDGE_CLIENT_ProcessCommandPacket(void)
{
    CFE_SB_MsgId_t  MsgId;

    MsgId = CFE_SB_GetMsgId(RACS2_UserMsgPkt_Ptr);
    OS_printf("RACS2_BRIDGE_CLIENT: recceived MsgId = %x\n", MsgId);

    racs2_user_msg_t* tmp_ptr;

    switch (MsgId)
    {
        case RACS2_BRIDGE_CLIENT_CMD_MID:
            RACS2_BRIDGE_CLIENT_ProcessGroundCommand();
            break;

        case RACS2_BRIDGE_CLIENT_SEND_HK_MID:
            RACS2_BRIDGE_CLIENT_ReportHousekeeping();
            break;

        case RACS2_BRIDGE_MID:
            tmp_ptr = (racs2_user_msg_t*) RACS2_UserMsgPkt_Ptr;
            OS_printf("RACS2_BRIDGE_CLIENT: received sample message from talker, MID = [0x%x].\n",
                MsgId
            );
            OS_printf("RACS2_BRIDGE_CLIENT: ros2_topic_name = %s\n", tmp_ptr->ros2_topic_name);
            OS_printf("RACS2_BRIDGE_CLIENT: body_data_length = %d\n", tmp_ptr->body_data_length);

            pthread_mutex_lock(&g_bridge_msg_pkt_mutex);
            // Clear the message
            memset(&g_bridge_msg_pkt, 0, ROS2_TOPIC_NAME_LNGTH+BODY_DATA_MAX_LNGTH);
            // set topic name data
            memcpy(&g_bridge_msg_pkt[0], (uint8_t*)&tmp_ptr->ros2_topic_name, ROS2_TOPIC_NAME_LNGTH);
            // set body data
            memcpy(&g_bridge_msg_pkt[BRIDGE_HEADER_LNGTH], (uint8_t*)&tmp_ptr->body_data, tmp_ptr->body_data_length);
            g_should_send_msg_pkt = 1;
            pthread_mutex_unlock(&g_bridge_msg_pkt_mutex);

            break;

        default:
            RACS2_BRIDGE_CLIENT_HkTelemetryPkt.sample_command_error_count++;
            CFE_EVS_SendEvent(SAMPLE_COMMAND_ERR_EID,CFE_EVS_EventType_ERROR,
            "RACS2_BRIDGE_CLIENT: invalid command packet,MID = 0x%x", MsgId);
            break;
    }

    return;

} /* End RACS2_BRIDGE_CLIENT_ProcessCommandPacket */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * ***/
/*                                                                                     */
/* RACS2_BRIDGE_CLIENT_ProcessGroundCommand() -- RACS2_BRIDGE_CLIENT ground commands   */
/*                                                                                     */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * ***/

void RACS2_BRIDGE_CLIENT_ProcessGroundCommand(void)
{
    uint16 CommandCode;

    CommandCode = CFE_SB_GetCmdCode(RACS2_UserMsgPkt_Ptr);

    /* Process "known" RACS2_BRIDGE_CLIENT app ground commands */
    switch (CommandCode)
    {
        case SAMPLE_APP_NOOP_CC:
            RACS2_BRIDGE_CLIENT_HkTelemetryPkt.sample_command_count++;
            CFE_EVS_SendEvent(SAMPLE_COMMANDNOP_INF_EID,
                        CFE_EVS_EventType_INFORMATION,
                        "RACS2_BRIDGE_CLIENT: NOOP command");
            break;

        case SAMPLE_APP_RESET_COUNTERS_CC:
            RACS2_BRIDGE_CLIENT_ResetCounters();
            break;

        /* default case already found during FC vs length test */
        default:
            break;
    }
    return;

} /* End of RACS2_BRIDGE_CLIENT_ProcessGroundCommand() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  RACS2_BRIDGE_CLIENT_ReportHousekeeping                             */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function is triggered in response to a task telemetry request */
/*         from the housekeeping task. This function will gather the Apps     */
/*         telemetry, packetize it and send it to the housekeeping task via   */
/*         the software bus                                                   */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void RACS2_BRIDGE_CLIENT_ReportHousekeeping(void)
{
    CFE_SB_TimeStampMsg((CFE_SB_Msg_t *) &RACS2_BRIDGE_CLIENT_HkTelemetryPkt);
    CFE_SB_SendMsg((CFE_SB_Msg_t *) &RACS2_BRIDGE_CLIENT_HkTelemetryPkt);
    return;

} /* End of RACS2_BRIDGE_CLIENT_ReportHousekeeping() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  RACS2_BRIDGE_CLIENT_ResetCounters                                               */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function resets all the global counter variables that are     */
/*         part of the task telemetry.                                        */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void RACS2_BRIDGE_CLIENT_ResetCounters(void)
{
    /* Status of commands processed by the RACS2_BRIDGE_CLIENT App */
    RACS2_BRIDGE_CLIENT_HkTelemetryPkt.sample_command_count       = 0;
    RACS2_BRIDGE_CLIENT_HkTelemetryPkt.sample_command_error_count = 0;

    CFE_EVS_SendEvent(SAMPLE_COMMANDRST_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "RACS2_BRIDGE_CLIENT: RESET command");
    return;

} /* End of RACS2_BRIDGE_CLIENT_ResetCounters() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* RACS2_BRIDGE_CLIENT_VerifyCmdLength() -- Verify command packet length                   */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
bool RACS2_BRIDGE_CLIENT_VerifyCmdLength(CFE_SB_MsgPtr_t msg, uint16 ExpectedLength)
{
    bool result = true;

    uint16 ActualLength = CFE_SB_GetTotalMsgLength(msg);

    /*
    ** Verify the command packet length.
    */
    if (ExpectedLength != ActualLength)
    {
        CFE_SB_MsgId_t MessageID   = CFE_SB_GetMsgId(msg);
        uint16         CommandCode = CFE_SB_GetCmdCode(msg);

        CFE_EVS_SendEvent(SAMPLE_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
           "Invalid msg length: ID = 0x%X,  CC = %d, Len = %d, Expected = %d",
              MessageID, CommandCode, ActualLength, ExpectedLength);
        result = false;
        RACS2_BRIDGE_CLIENT_HkTelemetryPkt.sample_command_error_count++;
    }

    return(result);

} /* End of RACS2_BRIDGE_CLIENT_VerifyCmdLength() */

bool is_new_msgid(uint16 msgid)
{
    if (registerd_msgid_num >= RACS2_BRIDGE_DEST_MSGID_NUM) return false;

    int i = 0;
    // check if msgid is already registered
    for (i = 0; i < RACS2_BRIDGE_DEST_MSGID_NUM; i++)
    {
        if (msgid == dest_message_id_list[i])
        {
            return false;
        }
    }
    // register msgid
    dest_message_id_list[registerd_msgid_num] = msgid;
    registerd_msgid_num++;
    OS_printf("RACS2_BRIDGE_CLIENT: register msgid[%x], number of reigsterd ID[%d]\n", msgid, registerd_msgid_num);

    return true;
}
