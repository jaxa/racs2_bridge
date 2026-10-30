#include "cfe.h"
#include "racs2_user_msg.h"
#include <string.h>
void PROBE_Main(void) {
 CFE_SB_PipeId_t pipe; CFE_SB_MsgPtr_t msg; uint32 state=CFE_ES_APP_RUN;
 CFE_ES_RegisterApp();
 if(CFE_SB_CreatePipe(&pipe,16,"PROBE_PIPE")!=CFE_SUCCESS) {OS_printf("SINK_INIT_FAIL\n");return;}
 if(CFE_SB_Subscribe(0x1894,pipe)!=CFE_SUCCESS) {OS_printf("SINK_SUB_FAIL\n");return;}
 OS_printf("SINK_READY\n");
 while(CFE_ES_RunLoop(&state)) {
  if(CFE_SB_RcvMsg(&msg,pipe,500)==CFE_SUCCESS) {
   racs2_user_msg_t *p=(racs2_user_msg_t *)msg;
   unsigned n=p->body_data_length;int ok=n<=128;
   for(unsigned i=0;ok && i<n;i++) if(p->body_data[i]!=(unsigned char)(i%251))ok=0;
   OS_printf("SINK length=%u match=%d\n",n,ok);
  }
 }
 CFE_ES_ExitApp(state);
}
