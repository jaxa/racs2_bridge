FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update -qq && apt-get install -y --no-install-recommends git cmake build-essential libwebsockets-dev protobuf-c-compiler libprotobuf-c-dev python3 python3-pip ca-certificates && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --no-cache-dir websockets==12.0
RUN git clone https://github.com/nasa/cFS.git /opt/cFS && cd /opt/cFS && git checkout 3684ce90fbd62b92d894e1c30e9eeef749ea02d0 && git submodule update --init --recursive
RUN git clone https://github.com/jaxa/racs2_bridge.git /opt/baseline && cd /opt/baseline && git checkout 8233f2fdf8df59c09016cd8e440c5c2b092ff928
ARG VARIANT=patched
COPY . /opt/candidate
RUN cp -r /opt/cFS/cfe/cmake/sample_defs /opt/cFS/sample_defs && cp -r /opt/baseline/cFS/Bridge/Client_C/apps/racs2_bridge_client /opt/cFS/apps/ && cp /opt/baseline/cFS/Bridge/Client_C/sample_defs/* /opt/cFS/sample_defs/ && if [ "$VARIANT" = patched ]; then cp /opt/candidate/cFS/Bridge/Client_C/apps/racs2_bridge_client/fsw/src/racs2_bridge_client.c /opt/cFS/apps/racs2_bridge_client/fsw/src/; elif [ "$VARIANT" != baseline ]; then exit 1; fi
RUN mkdir -p /opt/cFS/apps/probe && cp /opt/candidate/tests/integration/probe.c /opt/cFS/apps/probe/ && cp /opt/candidate/tests/integration/CMakeLists.txt /opt/cFS/apps/probe/ && python3 /opt/candidate/tests/integration/configure.py && cmake -S /opt/cFS/cfe -B /opt/cFS/build -DSIMULATION=native && cmake --build /opt/cFS/build --target mission-all -j2 && cmake --build /opt/cFS/build --target mission-install
CMD ["python3", "/opt/candidate/tests/integration/downstream.py"]
