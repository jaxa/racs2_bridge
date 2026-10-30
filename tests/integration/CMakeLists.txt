cmake_minimum_required(VERSION 2.6.4)
project(probe C)
include_directories(../racs2_bridge_client/fsw/src)
add_cfe_app(probe probe.c)
