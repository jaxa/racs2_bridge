# Real cFS integration

From the repository root:

```sh
docker build -f tests/integration/Dockerfile --build-arg VARIANT=patched -t racs2-test:patched .
docker build -f tests/integration/Dockerfile --build-arg VARIANT=baseline -t racs2-test:baseline .
docker run --rm --network none --cap-add SYS_NICE --ulimit rtprio=99 --ulimit msgqueue=8192000 --sysctl fs.mqueue.msg_max=256 racs2-test:patched
```

The patched test requires three exact 128-byte payload matches at a real cFS Software Bus subscriber: whole message, fragmented message, then delivery after an oversized message closes the connection and the client reconnects. Nonzero exit means failure; retain the container (omit --rm) and copy /tmp/downstream.log for diagnosis.

Run the same normal/fragmented comparison for each image:

```sh
docker run --rm --network none --cap-add SYS_NICE --ulimit rtprio=99 --ulimit msgqueue=8192000 --sysctl fs.mqueue.msg_max=256 racs2-test:baseline python3 /opt/candidate/tests/integration/comparison.py
docker run --rm --network none --cap-add SYS_NICE --ulimit rtprio=99 --ulimit msgqueue=8192000 --sysctl fs.mqueue.msg_max=256 racs2-test:patched python3 /opt/candidate/tests/integration/comparison.py
```

Expected patched body lengths: [96,96]. The baseline has exhibited [96,240,80,224]; callback splits are transport-dependent, so do not require that exact erroneous sequence on every host. Comparison JSON alone is observational; downstream.py performs assertions.

cFS and baseline repository commits and Python websockets version are fixed. Ubuntu packages and the image tag are not digest-locked, so bit-for-bit build reproducibility is not claimed. Tested scope is local Linux, not flight hardware. NOVA is optional external evidence tooling and is not a build dependency.
