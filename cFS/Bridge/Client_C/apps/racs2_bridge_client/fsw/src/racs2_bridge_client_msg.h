#ifndef _racs2_bridge_client_msg_h_
#define _racs2_bridge_client_msg_h_

/*
** SAMPLE App command codes
*/
#define SAMPLE_APP_NOOP_CC                 0
#define SAMPLE_APP_RESET_COUNTERS_CC       1

/*************************************************************************/
/*
** Type definition (generic "no arguments" command)
*/
typedef struct
{
   CFE_MSG_CommandHeader_t CmdHeader;

} SAMPLE_NoArgsCmd_t;

/*************************************************************************/
/*
** Type definition (SAMPLE App housekeeping)
*/
typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader;
    uint8              sample_command_error_count;
    uint8              sample_command_count;
    uint8              spare[2];

} sample_hk_tlm_t;

#define SAMPLE_APP_HK_TLM_LNGTH   sizeof ( sample_hk_tlm_t )


/*
** Type definition (char array data)
*/
typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader;
    uint8              sample_command_error_count;
    uint8              sample_command_count;
    char               char_arr[64];

} cfs_char_arr_msg_t;

#define CFS_CHAR_ARR_LNGTH   64
#define CFS_CHAR_ARR_MSG_LNGTH   sizeof ( cfs_char_arr_msg_t )
#define RACS2_BRIDGE_USER_LISTENER_LNGTH   sizeof ( cfs_char_arr_msg_t )

#endif /* _racs2_bridge_client_msg_h_ */
