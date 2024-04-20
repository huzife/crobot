cmake_minimum_required(VERSION 3.0.2)
project(crobot)

find_package(CSerialPort REQUIRED)

include_directories(include ${CSerialPort_INCLUDE_DIR})

add_compile_options(-O2)

add_library(crobot SHARED
    src/controller.cpp
    src/data_parser.cpp
    src/listener.cpp
    src/message/request.cpp
    src/message/response.cpp
    src/message/utils.cpp
)

target_link_libraries(crobot ${CSerialPort_LIBRARY})
