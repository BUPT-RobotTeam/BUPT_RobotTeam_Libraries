cmake_minimum_required(VERSION 3.24)

option(USE_SimpleLib "开启SimpleLib" OFF)
option(USE_MotorLib "开启MotorLib" OFF)
option(USE_ChassisLib "开启ChassisLib" OFF)

set(USE_MotorLib ON)
set(USE_ChassisLib ON)

set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/BUPTLib_port.h)
set(INC ${CMAKE_CURRENT_SOURCE_DIR}/)

if (USE_SimpleLib)
    add_subdirectory(SimpleLib)
#    list(BUPTLib APPEND ${})
endif ()

if (USE_MotorLib)
    message("Added MotorLib")
    add_subdirectory(MotorLib)
    list(APPEND SRC  ${MotorLib})
    list(APPEND INC  ${MotorLib_Inc})
endif ()

if (USE_ChassisLib)
    message("Added ChassisLib")
#    add_subdirectory(ChassisLib)
#    list(APPEND SRC ${ChassisLib})
#    list(APPEND INC ${ChassisLib_INC})
    add_subdirectory(ChassisLib_CXX)
    list(APPEND SRC ${ChassisLib})
    list(APPEND INC ${ChassisLib_INC})

endif ()

set(BUPTLib ${SRC} PARENT_SCOPE)
set(BUPTLib_Inc ${INC} PARENT_SCOPE)