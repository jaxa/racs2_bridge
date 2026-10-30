"""Compile the real receive-case body with cFS/logging stubs under ASan."""
from pathlib import Path
import subprocess
import tempfile

SOURCE = Path(__file__).parents[1] / 'cFS/Bridge/Client_C/apps/racs2_bridge_client/fsw/src/racs2_bridge_client.c'

def run_receive_case(text):
    body = text.split('case LWS_CALLBACK_CLIENT_RECEIVE:', 1)[1].split('case LWS_CALLBACK_CLIENT_WRITEABLE:', 1)[0]
    preamble = r'''
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
typedef int32_t int32;
typedef void CFE_SB_Msg_t;
typedef void *CFE_SB_MsgPtr_t;
#define RACS2_BRIDGE_HEADER_LENGTH 32
#define BODY_DATA_MAX_LNGTH 128
#define RACS2_USER_MSG_LNGTH sizeof(RACS2_UserMsgPkt)
#define CFE_SUCCESS 0
#define lwsl_user(...) printf(__VA_ARGS__)
#define lwsl_err(...) printf(__VA_ARGS__)
#define OS_printf(...) printf(__VA_ARGS__)
static struct {uint8_t body_data_length; uint8_t body_data[128];} RACS2_UserMsgPkt;
static int sent, last_id;
static void CFE_SB_InitMsg(void *p, unsigned id, size_t size, int clear) {
    last_id=id; memset(p, 0, size);
}
static void CFE_SB_TimeStampMsg(void *p) {(void)p;}
static int CFE_SB_SendMsg(void *p) {(void)p; sent++; return 0;}
static int CFE_SB_GetMsgId(void *p) {(void)p; return last_id;}
#define true 1
struct receive_state {uint8_t data[160];size_t used;};
struct lws {int final;size_t remaining;};
static int lws_is_final_fragment(struct lws *w){return w->final;}
static size_t lws_remaining_packet_payload(struct lws *w){return w->remaining;}
static size_t pending_bytes;
static int receive_fragment(void *in, size_t len, struct receive_state *rx, int final) {
struct lws socket={final,pending_bytes};struct lws *wsi=&socket;void *user=rx;
switch(1) {case 1:
'''
    suffix = r'''
}return 0;}
static void receive(void *in,size_t len){struct receive_state rx={0};receive_fragment(in,len,&rx,1);}
int main(void) {
    const size_t sizes[]={0,1,31,32,33,159,160,161,287,288};
    for(size_t n=0;n<sizeof(sizes)/sizeof(sizes[0]);n++) {
        size_t len=sizes[n]; uint8_t *p=malloc(len?len:1);
        memset(p, 0xa5, len?len:1); if(len>=2){p[0]=0x18;p[1]=0x94;}
        sent=0;receive(p,len);
        if(len>=32 && len<=160) {
            assert(sent==1 && last_id==0x1894);
            assert(RACS2_UserMsgPkt.body_data_length==len-32);
            assert(memcmp(RACS2_UserMsgPkt.body_data,p+32,len-32)==0);
        } else assert(sent==0);
        free(p);
    }
    {
        uint8_t packet[160];memset(packet,0xa5,sizeof(packet));packet[0]=0x18;packet[1]=0x94;
        struct receive_state rx={0};sent=0;
        assert(receive_fragment(packet,16,&rx,0)==0 && sent==0);
        assert(receive_fragment(packet+16,74,&rx,0)==0 && sent==0);
        assert(receive_fragment(packet+90,70,&rx,0)==0 && sent==0);
        assert(receive_fragment(packet,0,&rx,1)==0 && sent==1);
        assert(last_id==0x1894 && RACS2_UserMsgPkt.body_data_length==128);
        assert(memcmp(RACS2_UserMsgPkt.body_data,packet+32,128)==0);
        assert(rx.used==0);
        sent=0;assert(receive_fragment(packet,32,&rx,1)==0 && sent==1);
        struct receive_state overflow={0};sent=0;
        assert(receive_fragment(packet,160,&overflow,0)==0 && sent==0);
        assert(receive_fragment(packet,1,&overflow,1)==-1 && sent==0);
        struct receive_state partial={0},other={0};sent=0;
        pending_bytes=128;
        assert(receive_fragment(packet,32,&partial,1)==0 && sent==0);
        pending_bytes=0;
        assert(receive_fragment(packet,32,&other,1)==0 && sent==1);
        assert(partial.used==32 && other.used==0);
        assert(receive_fragment(packet+32,128,&partial,1)==0 && sent==2);
        assert(memcmp(RACS2_UserMsgPkt.body_data,packet+32,128)==0);
    }
    puts("Production receive boundary and fragment checks passed");
    return 0;
}
'''
    with tempfile.TemporaryDirectory(prefix='racs2-callback-') as folder:
        src=Path(folder)/'probe.c'; binary=Path(folder)/'probe'
        src.write_text(preamble+body+suffix)
        subprocess.run(['clang','-O0','-g','-fsanitize=address,undefined',str(src),'-o',str(binary)],check=True,capture_output=True)
        result=subprocess.run([str(binary)],capture_output=True,text=True,timeout=20)
        assert result.returncode==0, result.stdout+result.stderr
        assert 'runtime error:' not in result.stderr

def test_receive_case():
    run_receive_case(SOURCE.read_text())
