cmake_minimum_required(VERSION 3.16)

project(control_system_test C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_path(ETHERCAT_INCLUDE_DIR
  NAMES ecrt.h
  PATHS /usr/local/include /usr/include
)

find_library(ETHERCAT_LIBRARY
  NAMES ethercat
  PATHS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu
)

if(NOT ETHERCAT_INCLUDE_DIR OR NOT ETHERCAT_LIBRARY)
  message(FATAL_ERROR "Missing EtherCAT dependency: ecrt.h and/or libethercat. Expected in /usr/local or system include/lib paths.")
endif()

find_package(Threads REQUIRED)
find_library(RT_LIBRARY NAMES rt)

add_custom_target(compile_commands ALL
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_BINARY_DIR}/compile_commands.json
    ${CMAKE_SOURCE_DIR}/compile_commands.json
  BYPRODUCTS ${CMAKE_SOURCE_DIR}/compile_commands.json
)

add_executable(test_all
  src/test_all.c
)
target_include_directories(test_all PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ETHERCAT_INCLUDE_DIR}
)
target_link_libraries(test_all PRIVATE
  ${ETHERCAT_LIBRARY}
  Threads::Threads
  ${RT_LIBRARY}
)

add_executable(test_io_raw
  src/test_io_raw.c
)
target_compile_definitions(test_io_raw PRIVATE
  _POSIX_C_SOURCE=200809L
)
target_include_directories(test_io_raw PRIVATE
  ${ETHERCAT_INCLUDE_DIR}
)
target_link_libraries(test_io_raw PRIVATE
  ${ETHERCAT_LIBRARY}
  Threads::Threads
  ${RT_LIBRARY}
)
