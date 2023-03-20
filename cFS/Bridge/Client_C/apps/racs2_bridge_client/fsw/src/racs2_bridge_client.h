#ifndef _racs2_bridge_client_h_
#define _racs2_bridge_client_h_

/*
** Required header files.
*/
#include "cfe.h"
#include "cfe_error.h"
#include "cfe_evs.h"
#include "cfe_sb.h"
#include "cfe_es.h"

#include <string.h>
#include <errno.h>
#include <unistd.h>

/***********************************************************************/

#define SAMPLE_PIPE_DEPTH                     32

/************************************************************************
** Type Definitions
*************************************************************************/

/****************************************************************************/
/*
** Local function prototypes.
**
** Note: Except for the entry point (SAMPLE_AppMain), these
**       functions are not called from any other source module.
*/
void SAMPLE_AppMain(void);
void SAMPLE_AppInit(void);
void SAMPLE_LISTENER_ProcessCommandPacket(void);
void RACS2_BRIDGE_CLIENT_ProcessGroundCommand(void);
void RACS2_BRIDGE_CLIENT_ReportHousekeeping(void);
void RACS2_BRIDGE_CLIENT_ResetCounters(void);

bool RACS2_BRIDGE_CLIENT_VerifyCmdLength(CFE_SB_MsgPtr_t msg, uint16 ExpectedLength);

bool is_new_msgid(uint16 msgid);

#endif /* _racs2_bridge_client_h_ */
