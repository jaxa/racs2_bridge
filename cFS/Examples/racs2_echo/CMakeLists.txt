cmake_minimum_required(VERSION 3.16)
project(CFE_RACS2_ECHO C)
get_filename_component(RACS2_APP_DIR "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
get_filename_component(RACS2_ROOT "${RACS2_APP_DIR}/../../.." ABSOLUTE)
add_cfe_app(racs2_echo fsw/src/racs2_echo.c)
target_include_directories(racs2_echo PRIVATE "${RACS2_ROOT}/Common/include"
  "${RACS2_ROOT}/cFS/Bridge/Client_C/apps/racs2_bridge_client/fsw/platform_inc")
