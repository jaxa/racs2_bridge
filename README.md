# RACS2 (ROS and cFS System 2) Bridge

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
  - Ubuntu 20.04 LTS
- ROS2 :
  - "ros-foxy" package.
- cFS : 
  - [cFE 6.7.0a](https://github.com/nasa/cFS/releases/tag/v6.7.0a)
  - [OSAL v5.0.0](https://github.com/nasa/osal/releases/tag/v5.0.0)


## Setup

### Premise

- ROS2 and cFS shall be installed on the OS.
  - See below.
    - [ROS2 Installation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
    - [cFS Installation](https://github.com/nasa/cFE)
    - [OSAL Installation](https://github.com/nasa/osal)

### Procedure

- Preparing for WebSocket.
  - C/C++ :
    ```
    sudo apt install -y libwebsockets-dev
    ```
  - Python :
    ```
    pip install websockets
    ```

- Preparing for Protocol Buffers.
  - C/C++ :
    ```
    sudo apt install -y libwebsockets-dev protobuf-c-compiler libprotobuf-c-dev
    ```
  - Python :
    ```
    pip install protobuf
    ```

- git clone the [RACS2 Bridge](https://github.com/jaxa/racs2_bridge.git) source code.  

- Preparation of execution environment on the cFS side.  
  - Go to the top of the cFS project directory and execute the following build command
    ```
    cp cfe/cmake/Makefile.sample Makefile
    cp -r cfe/cmake/sample_defs sample_defs
    ```

  - Bridge application placement in the cFS execution environment.
    ```
    cp -pr racs2_bridge/cFS/Bridge/Client_C/apps/racs2_bridge_client [cFS project path]/apps/
    cp -p racs2_bridge/cFS/Bridge/Client_C/sample_defs/* [cFS project path]/sample_defs/
    ```

  - Go to the top of the cFS project directory and execute the following build command
    ```
    make prep
    make
    make install
    ```

- Preparation of execution environment on the ROS2 side.   
  - Bridge node placement in the ROS2 execution environment.
    ```
    cp -pr racs2_bridge/ROS2/Bridge/Server_Python/bridge_py_s [ROS2 project path]/src/
    ```
  - Go to the top of the ROS2 project directory and execute the following build command
    ```
    colcon build --symlink-install
    ```

- Start the ROS2 node.
  ```
  cd [ROS2 project path]
  source install/setup.bash
  ros2 run bridge_py_s bridge_py_s_node  --ros-args --params-file ./src/bridge_py_s/config/params.yaml
  ```

- Start the cFS application.
  ```
  cd [cFS project path]/build/exe/cpu1
  ./core-cpu1
  ```

- Start the ROS2 publishers and subscribers.

- Start the cFS publishers and subscribers.

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

## About Example

- See `Example/README.md`.

## How to exchange messages

- See `Document/HowToExchangeMessages.md`.

## Reference 

* [Hiroki Kato, et al. "ROS and cFS System (RACS): Easing Space Robotic Development ~post-opensource activities and ROS2 integration~" Flight Software Workshop 2021.](https://drive.google.com/file/d/11L48doT_pRNs7R0hdChPALqJO849TvV2/view?usp=drive_web)

