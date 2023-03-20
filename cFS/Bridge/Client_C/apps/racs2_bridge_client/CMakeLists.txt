cmake_minimum_required(VERSION 2.6.4)
project(CFE_RACS2_BRIDGE_CLIENT C)

include_directories(fsw/mission_inc)
include_directories(fsw/platform_inc)

aux_source_directory(fsw/src APP_SRC_FILES)

# Create the app module
add_cfe_app(racs2_bridge_client ${APP_SRC_FILES})

target_link_libraries(racs2_bridge_client
  websockets
)