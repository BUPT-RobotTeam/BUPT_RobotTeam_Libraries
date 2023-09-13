cmake_minimum_required(VERSION 3.24)

set(ChassisType rudder_chassis omni_chassis )
message("Used Chassis Type : ${ChassisType}")
set(
    ChassisLib_SRC
    #BASE
    ${CMAKE_CURRENT_SOURCE_DIR}/base_chassis.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/base_chassis.h
    ${CMAKE_CURRENT_SOURCE_DIR}/point.h
    ${CMAKE_CURRENT_SOURCE_DIR}/point.c
    ${CMAKE_CURRENT_SOURCE_DIR}/chassis_common_config.h
#    ${CMAKE_CURRENT_SOURCE_DIR}/base_chassis.h
#    ${CMAKE_CURRENT_SOURCE_DIR}/handle.h
#    ${CMAKE_CURRENT_SOURCE_DIR}/handle.c
#    ${CMAKE_CURRENT_SOURCE_DIR}/point_parser.h
#    ${CMAKE_CURRENT_SOURCE_DIR}/point_parser.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ctrlPathTracker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ctrlPathTracker.h
    ${CMAKE_CURRENT_SOURCE_DIR}/ctrlGo2Point.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ctrlGo2Point.h
    ${CMAKE_CURRENT_SOURCE_DIR}/ctrlMacro.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ctrlMacro.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../SimpleLib/utils/utils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../SimpleLib/utils/utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../SimpleLib/utils/vec.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../SimpleLib/utils/vec.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../SimpleLib/utils/kalman.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../SimpleLib/utils/kalman.h
#    BUPT_RobotTeam_Libraries/SimpleLib/utils

    # apps
    ${CMAKE_CURRENT_SOURCE_DIR}/app_example/chassis_bridge.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/app_example/chassis_bridge.h
    ${CMAKE_CURRENT_SOURCE_DIR}/app_example/chassis_ctrl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/app_example/chassis_ctrl.h
    ${CMAKE_CURRENT_SOURCE_DIR}/app_example/chassis_cmd.h
)
if ( rudder_chassis IN_LIST ChassisType)
    LIST(
        APPEND ChassisLib_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/steer_wheel.h
        ${CMAKE_CURRENT_SOURCE_DIR}/steer_wheel.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/rudder_chassis.h
        ${CMAKE_CURRENT_SOURCE_DIR}/rudder_chassis.cpp
    )#RUDDER_SRC
endif ()
if( omni_chassis IN_LIST ChassisType)
    LIST(
        APPEND ChassisLib_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/omni_chassis.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/omni_chassis.h
    )#OMNISRC
endif ()
if (NOT ((rudder_chassis IN_LIST ChassisType) OR (omni_chassis IN_LIST ChassisType) ))
    message(FATAL_ERROR "CHASSIS TYPE WRONG! GOT:${ChassisType}")
endif ()

set(ChassisLib ${ChassisLib_SRC} PARENT_SCOPE)
set(
    ChassisLib_INC PARENT_SCOPE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../SimpleLib/utils
)
