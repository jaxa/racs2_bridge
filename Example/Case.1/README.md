# RACS2 (ROS and cFS System 2) Example - Case.1

## Overview

This file contains a description of the RACS2 Example. The following movements can be observed in this Example.
01. The Example Publisher node(app) throws a message to RACS2 Bridge (Server).
02. The received message is transmitted by the RACS2 Bridge (Server) to the opposite RACS2 Bridge (Client).
03. The RACS2 Bridge (Client) publishes the transmitted message to the The Example Subscribed node(app).

## Directory List

- ROS2/
  - subscriber_c2s
    - The Example Subscriber for ROS2.
  - racs2_msg
    - The definition of message for bridge.
- cFS/
  - apps/sample_talker
    - The Example Publisher for cFS.
  - sample_defs
    - The Example build settings for cFS.

## Setup

### Premise

- The RACS2 Bridge Server and Client must be up and connected.

### Procedure

- Preparation of execution environment on the ROS2 side.

  - Place ROS2 bridge and example directories in the ROS2 execution environment.
    ```
    cp -pr  racs2_bridge/ROS2/Bridge/Server_Python/bridge_py_s [ROS2 project path]/src/
    cp -pr  racs2_bridge/Example/Case.1/ROS2/subscriber_c2s [ROS2 project path]/src/
    cp -pr  racs2_bridge/Example/Case.1/ROS2/racs2_msg [ROS2 project path]/src/
    ```

  - Go to the top of the ROS2 project directory and execute the following build command.
    ```
    colcon build --symlink-install
    ```

- Preparation of execution environment on the cFS side.

  - Go to the top of the cFS project directory and execute the following build command.
    ```
    cp cfe/cmake/Makefile.sample Makefile
    cp -r cfe/cmake/sample_defs sample_defs
    ```

  - Place cFS bridge and example directories in the cFS execution environment.
    ```
    cp -pr  racs2_bridge/cFS/Bridge/Client_C/apps/racs2_bridge_client [cFS project path]/apps/
    cp -pr  racs2_bridge/Example/Case.1/cFS/apps/sample_talker [cFS project path]/apps/
    cp -pr  racs2_bridge/Example/Case.1/cFS/sample_defs/* [cFS project path]/sample_defs/
    ```

  - Edit L.250 of "[cFS project path]/sample_defs/default_osconfig.h" as follows,
    ```
    #define OSAL_DEBUG_PERMISSIVE_MODE
    ```

  - Go to the top of the cFS project directory and execute the following build command.
    ```
    make prep
    make
    make install
    ```

- Start the bridge nodes.
  ```
  cd [ROS2 project path]
  source install/setup.bash
  ros2 run bridge_py_s bridge_py_s_node  --ros-args --params-file ./src/bridge_py_s/config/params.yaml
  ```

- Start the ROS2 nodes.
  ```
  cd [ROS2 project path]
  source install/setup.bash
  ros2 run  subscriber_c2s  subscriber_c2s_node
  ```

- Start the cFS applications, talker.
  ```
  cd [cFS project path]/build/exe/cpu1
  ./core-cpu1
  ```

- Check the messages that have been published and subscribed.

## How to exchange messages

- See `Document/HowToExchangeMessages.md`.

