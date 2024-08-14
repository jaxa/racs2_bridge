/*
**   Include Files:
*/

#include "sample_talker.h"
#include "sample_talker_perfids.h"
#include "sample_talker_msgids.h"
#include "sample_talker_msg.h"
#include "racs2_user_msg.h"
#include "sample_talker_events.h"
#include "sample_talker_version.h"
#include "RACS2Brdige_std_msgs.pb-c.h"

/*
** global data
*/

sample_hk_tlm_t    SAMPLE_TALKER_HkTelemetryPkt;
racs2_user_msg_t   RACS2_UserMsgPkt;
CFE_SB_PipeId_t    SAMPLE_TALKER_CommandPipe;
CFE_SB_MsgPtr_t    SAMPLE_TALKER_MsgPtr;

static CFE_EVS_BinFilter_t  SAMPLE_EventFilters[] =
       {  /* Event ID    mask */
          {SAMPLE_STARTUP_INF_EID,       0x0000},
          {SAMPLE_COMMAND_ERR_EID,       0x0000},
          {SAMPLE_COMMANDNOP_INF_EID,    0x0000},
          {SAMPLE_COMMANDRST_INF_EID,    0x0000},
       };

/** * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* SAMPLE_TALKER_Main() -- Application entry point and main process loop          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
void SAMPLE_TALKER_Main( void )
{
    int32  status;
    uint32 RunStatus = CFE_ES_RunStatus_APP_RUN;

    OS_printf("SAMPLE_TALKER_Main starts.\n");

    CFE_ES_PerfLogEntry(SAMPLE_APP_PERF_ID);

    SAMPLE_TAKLKER_Init();

    /*
    ** SAMPLE_TALKER Runloop
    */
    int count = 0;
    while (CFE_ES_RunLoop(&RunStatus) == true)
    {
        CFE_ES_PerfLogExit(SAMPLE_APP_PERF_ID);

        // /* Pend on receipt of command packet -- timeout set to 500 millisecs */
        // status = CFE_SB_RcvMsg(&SAMPLE_TALKER_MsgPtr, SAMPLE_TALKER_CommandPipe, 500);

        CFE_ES_PerfLogEntry(SAMPLE_APP_PERF_ID);

        // if (status == CFE_SUCCESS)
        // {
        //     SAMPLE_TALKER_ProcessCommandPacket();
        // }
        sleep(2);

        // send message
        // set topic name
        strcpy(RACS2_UserMsgPkt.ros2_topic_name, "/Recv/RACS2Bridge");
        // define serialized body data
        void *buffer;
        int len=0;
        RACS2BridgeStdMsgs *message;
        message=(RACS2BridgeStdMsgs *)malloc(sizeof(RACS2BridgeStdMsgs));
        racs2_bridge_std_msgs__init(message);
        int string_length = 22;
        char* buf[32];
        sprintf(buf, "Message To ROS2 :%5d", count);
        message->string_data = (char *)malloc(sizeof(string_length));
        OS_printf("SAMPLE_TALKER: [Send][MsgID=0x%x][%s]\n", RACS2_BRIDGE_MID, buf);
        strncpy(message->string_data, buf, string_length);

        len = racs2_bridge_std_msgs__get_packed_size(message);
        buffer=malloc(len);
        racs2_bridge_std_msgs__pack(message, buffer);

        // set body data
        strncpy(RACS2_UserMsgPkt.body_data, buffer, len);
        // set body data length
        RACS2_UserMsgPkt.body_data_length = len;

        // send data
        CFE_SB_TimeStampMsg((CFE_SB_Msg_t *) &RACS2_UserMsgPkt);
        status = CFE_SB_SendMsg((CFE_SB_Msg_t *) &RACS2_UserMsgPkt);
        // OS_printf("SAMPLE_TALKER: Sent message, MID = [0x%x], sample_command_count = %d\n",
        //     CFE_SB_GetMsgId((CFE_SB_MsgPtr_t) &RACS2_UserMsgPkt),
        //     RACS2_UserMsgPkt.sample_command_count
        //     );
        if (status != CFE_SUCCESS) {
            OS_printf("SAMPLE_TALKER: Error: sending is failed. status = 0x%x\n", status);
        }

        free(buffer);
        free(message->string_data);
        free(message);
        memset(buf, '\0', sizeof(buf));

        count++;
    }

    CFE_ES_ExitApp(RunStatus);

} /* End of SAMPLE_TALKER_Main() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  */
/*                                                                            */
/* SAMPLE_TAKLKER_Init() --  initialization                                   */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void SAMPLE_TAKLKER_Init(void)
{
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

    SAMPLE_TALKER_ResetCounters();

    CFE_SB_InitMsg(&SAMPLE_TALKER_HkTelemetryPkt,
                   SAMPLE_TALKER_HK_TLM_MID,
                   SAMPLE_APP_HK_TLM_LNGTH, true);

    CFE_SB_InitMsg(&RACS2_UserMsgPkt, RACS2_BRIDGE_MID, RACS2_USER_MSG_LNGTH, false);

    CFE_EVS_SendEvent (SAMPLE_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION,
               "SAMPLE_TALKER App Initialized. Version %d.%d.%d.%d",
                SAMPLE_APP_MAJOR_VERSION,
                SAMPLE_APP_MINOR_VERSION,
                SAMPLE_APP_REVISION,
                SAMPLE_APP_MISSION_REV);

} /* End of SAMPLE_TAKLKER_Init() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  SAMPLE_TALKER_ProcessCommandPacket                                 */
/*                                                                            */
/*  Purpose:                                                                  */
/*     This routine will process any packet that is received on the SAMPLE_TALKER */
/*     command pipe.                                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void SAMPLE_TALKER_ProcessCommandPacket(void)
{
    CFE_SB_MsgId_t  MsgId;

    MsgId = CFE_SB_GetMsgId(SAMPLE_TALKER_MsgPtr);

    switch (MsgId)
    {
        case SAMPLE_TALKER_CMD_MID:
            SAMPLE_TALKER_ProcessGroundCommand();
            break;

        case SAMPLE_TALKER_SEND_HK_MID:
            SAMPLE_TALKER_ReportHousekeeping();
            break;

        default:
            SAMPLE_TALKER_HkTelemetryPkt.sample_command_error_count++;
            CFE_EVS_SendEvent(SAMPLE_COMMAND_ERR_EID,CFE_EVS_EventType_ERROR,
            "SAMPLE_TALKER: invalid command packet,MID = 0x%x", MsgId);
            break;
    }

    return;

} /* End SAMPLE_TALKER_ProcessCommandPacket */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* SAMPLE_TALKER_ProcessGroundCommand() -- SAMPLE_TALKER ground commands      */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/

void SAMPLE_TALKER_ProcessGroundCommand(void)
{
    uint16 CommandCode;

    CommandCode = CFE_SB_GetCmdCode(SAMPLE_TALKER_MsgPtr);

    /* Process "known" SAMPLE_TALKER app ground commands */
    switch (CommandCode)
    {
        case SAMPLE_APP_NOOP_CC:
            SAMPLE_TALKER_HkTelemetryPkt.sample_command_count++;
            CFE_EVS_SendEvent(SAMPLE_COMMANDNOP_INF_EID,
                        CFE_EVS_EventType_INFORMATION,
            "SAMPLE_TALKER: NOOP command");
            break;

        case SAMPLE_APP_RESET_COUNTERS_CC:
            SAMPLE_TALKER_ResetCounters();
            break;

        /* default case already found during FC vs length test */
        default:
            break;
    }
    return;

} /* End of SAMPLE_TALKER_ProcessGroundCommand() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  SAMPLE_TALKER_ReportHousekeeping                                   */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function is triggered in response to a task telemetry request */
/*         from the housekeeping task. This function will gather the Apps     */
/*         telemetry, packetize it and send it to the housekeeping task via   */
/*         the software bus                                                   */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void SAMPLE_TALKER_ReportHousekeeping(void)
{
    CFE_SB_TimeStampMsg((CFE_SB_Msg_t *) &SAMPLE_TALKER_HkTelemetryPkt);
    CFE_SB_SendMsg((CFE_SB_Msg_t *) &SAMPLE_TALKER_HkTelemetryPkt);
    return;

} /* End of SAMPLE_TALKER_ReportHousekeeping() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  SAMPLE_TALKER_ResetCounters                                        */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function resets all the global counter variables that are     */
/*         part of the task telemetry.                                        */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void SAMPLE_TALKER_ResetCounters(void)
{
    /* Status of commands processed by the SAMPLE_TALKER App */
    SAMPLE_TALKER_HkTelemetryPkt.sample_command_count       = 0;
    SAMPLE_TALKER_HkTelemetryPkt.sample_command_error_count = 0;

    CFE_EVS_SendEvent(SAMPLE_COMMANDRST_INF_EID, CFE_EVS_EventType_INFORMATION,
        "SAMPLE_TALKER: RESET command");
    return;

} /* End of SAMPLE_TALKER_ResetCounters() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* SAMPLE_TALKER_VerifyCmdLength() -- Verify command packet length            */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
bool SAMPLE_TALKER_VerifyCmdLength(CFE_SB_MsgPtr_t msg, uint16 ExpectedLength)
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
        SAMPLE_TALKER_HkTelemetryPkt.sample_command_error_count++;
    }

    return(result);

} /* End of SAMPLE_TALKER_VerifyCmdLength() */

