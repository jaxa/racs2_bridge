import asyncio,json,subprocess,re
import websockets
async def main():
 done=asyncio.Event(); connections=0
 async def handler(ws,path):
  nonlocal connections
  connections+=1
  await asyncio.sleep(2)
  packet=b'\x18\x94'+bytes(30)+bytes(i%251 for i in range(128))
  if connections==1:
   await ws.send(packet)
   await ws.send([packet[:16],packet[16:]])
   await asyncio.sleep(2)
   await ws.send(packet+b'x')
   try: await asyncio.wait_for(ws.recv(),4)
   except websockets.ConnectionClosed: pass
   else: raise AssertionError('oversize was not disconnected')
  else:
   await ws.send(packet)
   await asyncio.sleep(2);done.set();await asyncio.sleep(2)
 async with websockets.serve(handler,'127.0.0.1',8765,compression=None):
  with open('/tmp/downstream.log','w') as log:
   p=subprocess.Popen(['stdbuf','-oL','./core-cpu1'],cwd='/usr/local/cpu1',stdout=log,stderr=log)
   try: await asyncio.wait_for(done.wait(),30)
   finally:
    p.terminate()
    try:p.wait(timeout=3)
    except subprocess.TimeoutExpired:p.kill();p.wait()
 s=open('/tmp/downstream.log','rb').read().decode('latin1')
 rows=re.findall(r'SINK length=(\d+) match=(\d+)',s)
 assert rows==[('128','1')]*3,(rows,s[-2000:])
 assert 'receive message exceeds capacity' in s
 assert connections>=2
 print(json.dumps({'rows':rows,'connections':connections,'oversize_rejected':True}))
asyncio.run(main())
