import asyncio,json,subprocess,os
import websockets

async def main():
    done=asyncio.Event()
    async def handle(ws,path):
        await asyncio.sleep(2)
        packet=b'\x18\x94'+b'x'*126
        await ws.send(packet)
        await asyncio.sleep(.3)
        await ws.send([packet[:16],packet[16:]])
        await asyncio.sleep(3)
        done.set()
        await asyncio.sleep(2)
    async with websockets.serve(handle,'127.0.0.1',8765,compression=None):
        with open('/tmp/cfs-runtime.log','w') as log:
            proc=subprocess.Popen(['stdbuf','-oL','./core-cpu1'],cwd='/usr/local/cpu1',stdout=log,stderr=log)
            try: await asyncio.wait_for(done.wait(),15)
            finally:
                proc.terminate()
                try:proc.wait(timeout=3)
                except subprocess.TimeoutExpired:proc.kill();proc.wait()
    text=open('/tmp/cfs-runtime.log','rb').read().decode('latin1')
    import re
    lengths=[int(x) for x in re.findall(r'body data length : (\d+)',text)]
    print(json.dumps({'lengths':lengths,'initialized':'App Initialized' in text,'log':text}))
asyncio.run(main())
