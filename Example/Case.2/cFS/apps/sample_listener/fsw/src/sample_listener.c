/*
**   Include Files:
*/

#include "sample_listener.h"
#include "sample_listener_perfids.h"
#include "sample_listener_msgids.h"
#include "sample_listener_msg.h"
#include "racs2_user_msg.h"
#include "sample_listener_events.h"
#include "sample_listener_version.h"
#include "RACS2Bridge_std_msgs.pb-c.h"

/*
** global data
*/

sample_hk_tlm_t    SAMPLE_LISTENER_HkTelemetryPkt;
sample_hk_tlm_t    SAMPLE_TalkerListenerPkt;
CFE_SB_PipeId_t    SAMPLE_LISTENER_CommandPipe;
CFE_SB_MsgPtr_t    SAMPLE_LISTENER_MsgPtr;
racs2_user_msg_t   RACS2_UserMsgPkt;
CFE_SB_MsgPtr_t    RACS2_UserMsgPkt_Ptr;

static CFE_EVS_BinFilter_t  SAMPLE_EventFilters[] =
       {  /* Event ID    mask */
          {SAMPLE_STARTUP_INF_EID,       0x0000},
          {SAMPLE_COMMAND_ERR_EID,       0x0000},
          {SAMPLE_COMMANDNOP_INF_EID,    0x0000},
          {SAMPLE_COMMANDRST_INF_EID,    0x0000},
       };

/** * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* SAMPLE_LISTENER_Main() -- Application entry point and main process loop          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
void SAMPLE_LISTENER_Main( void )
{
    int32  status;
    uint32 RunStatus = CFE_ES_RunStatus_APP_RUN;

    OS_printf("SAMPLE_LISTENER_Main starts.\n");

    CFE_ES_PerfLogEntry(SAMPLE_APP_PERF_ID);

    SAMPLE_LISTENER_Init();

    /*
    ** SAMPLE_LISTENER Runloop
    */
    while (CFE_ES_RunLoop(&RunStatus) == true)
    {
        CFE_ES_PerfLogExit(SAMPLE_APP_PERF_ID);

        /* Pend on receipt of command packet -- timeout set to 500 millisecs */
        status = CFE_SB_RcvMsg(&RACS2_UserMsgPkt_Ptr, SAMPLE_LISTENER_CommandPipe, 1000);

        CFE_ES_PerfLogEntry(SAMPLE_APP_PERF_ID);

        if (status == CFE_SUCCESS)
        {
            SAMPLE_LISTENER_ProcessCommandPacket();
        }
        else
        {
            OS_printf("SAMPLE_LISTENER: CFE_SB_RcvMsg failed, status = 0x%x\n", status);
        }


    }

    CFE_ES_ExitApp(RunStatus);

} /* End of SAMPLE_LISTENER_Main() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  */
/*                                                                            */
/* SAMPLE_LISTENER_Init() --  initialization                                       */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void SAMPLE_LISTENER_Init(void)
{
    int32 status;
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
    status = CFE_SB_CreatePipe(&SAMPLE_LISTENER_CommandPipe, SAMPLE_PIPE_DEPTH, "TO_LISTENER");
    if (status != CFE_SUCCESS)
    {
        OS_printf("SAMPLE_LISTENER: failed to create SB pipe, status = 0x%x\n", status);
    }
    status = CFE_SB_Subscribe(SAMPLE_LISTENER_MID, SAMPLE_LISTENER_CommandPipe);
    if (status != CFE_SUCCESS)
    {
        OS_printf("SAMPLE_LISTENER: failed to subscribe MsgID[0x%x], status = 0x%x\n", SAMPLE_LISTENER_MID, status);
    }
    SAMPLE_LISTENER_ResetCounters();

    CFE_SB_InitMsg(&SAMPLE_LISTENER_HkTelemetryPkt,
                   SAMPLE_LISTENER_HK_TLM_MID,
                   SAMPLE_APP_HK_TLM_LNGTH, true);

    CFE_EVS_SendEvent (SAMPLE_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION,
               "SAMPLE_LISTENER App Initialized. Version %d.%d.%d.%d",
                SAMPLE_APP_MAJOR_VERSION,
                SAMPLE_APP_MINOR_VERSION,
                SAMPLE_APP_REVISION,
                SAMPLE_APP_MISSION_REV);

} /* End of SAMPLE_LISTENER_Init() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  SAMPLE_LISTENER_ProcessCommandPacket                                        */
/*                                                                            */
/*  Purpose:                                                                  */
/*     This routine will process any packet that is received on the SAMPLE_LISTENER    */
/*     command pipe.                                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void SAMPLE_LISTENER_ProcessCommandPacket(void)
{
    CFE_SB_MsgId_t  MsgId;

    MsgId = CFE_SB_GetMsgId(RACS2_UserMsgPkt_Ptr);
    OS_printf("SAMPLE_LISTENER: recceived MsgId = %x\n", MsgId);

    racs2_user_msg_t *tmpPtr;

    switch (MsgId)
    {
        case SAMPLE_LISTENER_CMD_MID:
            SAMPLE_LISTENER_ProcessGroundCommand();
            break;

        case SAMPLE_LISTENER_SEND_HK_MID:
            SAMPLE_LISTENER_ReportHousekeeping();
            break;

        case SAMPLE_LISTENER_MID:
            tmpPtr = (racs2_user_msg_t*) RACS2_UserMsgPkt_Ptr;
            OS_printf("SAMPLE_LISTENER: received sample message from talker, MID = [0x%x], Msg len = %d.\n",
                MsgId,
                // *(tmpPtr+CFE_SB_TLM_HDR_SIZE+ROS2_TOPIC_NAME_LNGTH+1)
                tmpPtr->body_data_length
                );
            OS_printf("SAMPLE_LISTENER: received sample message from talker, MID = [0x%x].\n",
                MsgId
            );
            char buffer[1024];
            int len=tmpPtr->body_data_length;

            RACS2BridgeStdMsgs *message;
            message = racs2_bridge_std_msgs__unpack(NULL, len, tmpPtr->body_data);
            if (!message)
            {
                OS_printf("SAMPLE_LISTENER: deserialization failed\n");
                break;
            }

            if (message->string_data)
            {
                OS_printf("SAMPLE_LISTENER: %s\n", message->string_data);
            }
            else
            {
                OS_printf("SAMPLE_LISTENER: data is empty\n");
            }

            racs2_bridge_std_msgs__free_unpacked(message, NULL);
            break;

        default:
            SAMPLE_LISTENER_HkTelemetryPkt.sample_command_error_count++;
            CFE_EVS_SendEvent(SAMPLE_COMMAND_ERR_EID,CFE_EVS_EventType_ERROR,
            "SAMPLE_LISTENER: invalid command packet,MID = 0x%x", MsgId);
            break;
    }

    return;

} /* End SAMPLE_LISTENER_ProcessCommandPacket */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* SAMPLE_LISTENER_ProcessGroundCommand() -- SAMPLE_LISTENER ground commands                    */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/

void SAMPLE_LISTENER_ProcessGroundCommand(void)
{
    uint16 CommandCode;

    // CommandCode = CFE_SB_GetCmdCode(SAMPLE_LISTENER_MsgPtr);
    CommandCode = CFE_SB_GetCmdCode(RACS2_UserMsgPkt_Ptr);

    /* Process "known" SAMPLE_LISTENER app ground commands */
    switch (CommandCode)
    {
        case SAMPLE_APP_NOOP_CC:
            SAMPLE_LISTENER_HkTelemetryPkt.sample_command_count++;
            CFE_EVS_SendEvent(SAMPLE_COMMANDNOP_INF_EID,
                        CFE_EVS_EventType_INFORMATION,
                        "SAMPLE_LISTENER: NOOP command");
            break;

        case SAMPLE_APP_RESET_COUNTERS_CC:
            SAMPLE_LISTENER_ResetCounters();
            break;

        /* default case already found during FC vs length test */
        default:
            break;
    }
    return;

} /* End of SAMPLE_LISTENER_ProcessGroundCommand() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  SAMPLE_LISTENER_ReportHousekeeping                                              */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function is triggered in response to a task telemetry request */
/*         from the housekeeping task. This function will gather the Apps     */
/*         telemetry, packetize it and send it to the housekeeping task via   */
/*         the software bus                                                   */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void SAMPLE_LISTENER_ReportHousekeeping(void)
{
    CFE_SB_TimeStampMsg((CFE_SB_Msg_t *) &SAMPLE_LISTENER_HkTelemetryPkt);
    CFE_SB_SendMsg((CFE_SB_Msg_t *) &SAMPLE_LISTENER_HkTelemetryPkt);
    return;

} /* End of SAMPLE_LISTENER_ReportHousekeeping() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  SAMPLE_LISTENER_ResetCounters                                               */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function resets all the global counter variables that are     */
/*         part of the task telemetry.                                        */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void SAMPLE_LISTENER_ResetCounters(void)
{
    /* Status of commands processed by the SAMPLE_LISTENER App */
    SAMPLE_LISTENER_HkTelemetryPkt.sample_command_count       = 0;
    SAMPLE_LISTENER_HkTelemetryPkt.sample_command_error_count = 0;

    CFE_EVS_SendEvent(SAMPLE_COMMANDRST_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "SAMPLE_LISTENER: RESET command");
    return;

} /* End of SAMPLE_LISTENER_ResetCounters() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* SAMPLE_LISTENER_VerifyCmdLength() -- Verify command packet length                   */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
bool SAMPLE_LISTENER_VerifyCmdLength(CFE_SB_MsgPtr_t msg, uint16 ExpectedLength)
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
        SAMPLE_LISTENER_HkTelemetryPkt.sample_command_error_count++;
    }

    return(result);

} /* End of SAMPLE_LISTENER_VerifyCmdLength() */

