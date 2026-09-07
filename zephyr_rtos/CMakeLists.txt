cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ELYOS_MotorDriver_Zephyr)

zephyr_include_directories(include)

target_sources(app PRIVATE 
    src/main.cpp
    src/foc_math.cpp
)
