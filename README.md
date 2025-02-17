# RACS2 (ROS2 and cFS System) Bridge

## Overview

This repository is a collection of software and examples that bridge message communication between Robot Operating System 2 (ROS2) nodes and core flight executives (cFE).  cFE is a core component of NASA-supplied spacecraft software, Core Flight System (CFS).

This project provides the following 2 components,

- ROS2/cFE message bridge software
- ROS2/cFE examples

**If you would like to report a bug or have a request for an update, please contact us. We will respond on demand.**

## License

This project is under the Apache 2.0 license. See LICENSE text file.

## Release Notes

- RACS2 (ROS2/cFE) project suite 1.0.0 is released.

## Directory List

- ROS2 :
  - RACS2 Bridge software for ROS2.
- cFS :
  - RACS2 Bridge software for cFS.
- Document :
  - Documents of RACS2.
- Example :
  - Example software of RACS2.
- Misc :
  - Other Information of RACS2.

## Dependency

- Base OS :
  - Ubuntu 22.04 LTS
- ROS2 :
  - "ros-humble" package.
- cFS :
  - [cFE 6.7.0a](https://github.com/nasa/cFS/releases/tag/v6.7.0a)
  - [OSAL v5.0.0](https://github.com/nasa/osal/releases/tag/v5.0.0) (included in cFE 6.7.0a)


## Setup

### Premise

- Assuming to work in the "~/racs2_ws" directory.
      ```
      mkdir ~/racs2_ws
      mkdir ~/racs2_ws/ros2_ws
      ```

- ROS2 and cFS shall be installed on the OS.
  - See below.
    - [ROS2 Installation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

    - [cFS Installation](https://github.com/nasa/cFS)
      ```
      cd ~/racs2_ws  
      git clone https://github.com/nasa/cFS.git
      cd cFS
      git checkout v6.7.0a
      git submodule init
      git submodule update
      ```

### Procedure for initial build 

- Preparing for WebSocket.
  - C/C++ :
    ```
    sudo apt install -y libwebsockets-dev
    ```
  - Python :
    ```
    pip install websockets==12.0
    ```

- Preparing for Protocol Buffers.
  - C/C++ :
    ```
    sudo apt install -y libwebsockets-dev protobuf-c-compiler libprotobuf-c-dev
    ```
  - Python :
    ```
    pip install protobuf==3.20.0 
    ```

- git clone the RACS2 Bridge.
  ```
  cd ~/racs2_ws
  git clone https://github.com/jaxa/racs2_bridge.git -b v1.1
  ```

  - Go to the top of the cFS project directory and execute the following build command
    ```
    cd ~/racs2_ws/cFS
    cp cfe/cmake/Makefile.sample Makefile
    cp -r cfe/cmake/sample_defs sample_defs
    ```

  - Bridge application placement in the cFS execution environment.
    ```
    cd ~/racs2_ws/
    cp -pr racs2_bridge/cFS/Bridge/Client_C/apps/racs2_bridge_client cFS/apps/
    cp -pr racs2_bridge/cFS/Bridge/Client_C/sample_defs/* cFS/sample_defs/
    ```

  - Edit L.205 of "cFS/sample_defs/default_osconfig.h" as follows,
    ```
    #define OSAL_DEBUG_PERMISSIVE_MODE
    ```

  - Go to the top of the cFS project directory and execute the following build command
    ```
    cd ~/racs2_ws/cFS
    make prep
    make
    make install
    ```

- Preparation of execution environment on the ROS2 side.
  - Bridge node placement in the ROS2 execution environment.
    ```
    cd ~/racs2_ws/
    cp -pr racs2_bridge/ROS2/Bridge/Server_Python/bridge_py_s ros2_ws/src/
    ```
  - Go to the top of the ROS2 project directory and execute the following build command
    ```
    cd ros2_ws/
    colcon build --symlink-install
    ```

### Procedure for use

It must be built separately for both cFS and ROS 2. For detailed procedure, please see the [example](https://github.com/jaxa/racs2_bridge/tree/main/Example). 

## Params for ROS2

- Specify startup parameters in YAML format. The name of the file describing the parameters is `params.yaml`.
- Parameters Details:
  - wss_uri  -> HostName or IP Address. The type is String.
  - wss_port -> Port Number. The type is Integer.

## Params for cFS

- Specify startup parameters in txt format. The name of the file describing the parameters is `racs2_bridge_config.txt`.
- Parameters Details:
  - wss_uri  -> HostName or IP Address.
  - wss_port -> Port Number.

## How to exchange messages

- See `Document/HowToExchangeMessages.md`.

## Reference

* [Hiroki Kato (JAXA) and Tatsuhiko Saito (Systems Engineering Consultants (SEC)). “RACS2: the ROS2 and cFS System, launched” Flight Software Workshop 2023 in March 2023](https://drive.google.com/file/d/1VBsiUEW6Z8pG8LvbM7lEyZMRMz9w-sjX/view)

