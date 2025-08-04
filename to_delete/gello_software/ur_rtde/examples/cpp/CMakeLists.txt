cmake_minimum_required(VERSION 3.5)
project(ur_rtde_examples)

set(DIR ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "${DIR}")
find_package(ur_rtde REQUIRED PATHS "${DIR}/../../Build/ur_rtde" "${DIR}/../../build/ur_rtde")

add_executable(forcemode_example forcemode_example.cpp)
target_link_libraries(forcemode_example PUBLIC ur_rtde::rtde)

add_executable(move_until_contact move_until_contact.cpp)
target_link_libraries(move_until_contact PUBLIC ur_rtde::rtde)

add_executable(io io_example.cpp)
target_link_libraries(io PUBLIC ur_rtde::rtde)

add_executable(move_path_async_example move_path_async_example.cpp)
target_link_libraries(move_path_async_example PUBLIC ur_rtde::rtde)

add_executable(movej_path_with_blend movej_path_with_blend_example.cpp)
target_link_libraries(movej_path_with_blend PUBLIC ur_rtde::rtde)

add_executable(servoj servoj_example.cpp)
target_link_libraries(servoj PUBLIC ur_rtde::rtde)

add_executable(speedj speedj_example.cpp)
target_link_libraries(speedj PUBLIC ur_rtde::rtde)

add_executable(move_async move_async_example.cpp)
target_link_libraries(move_async PUBLIC ur_rtde::rtde)

add_executable(robotiq_gripper_example robotiq_gripper_example.cpp)
target_link_libraries(robotiq_gripper_example PUBLIC ur_rtde::rtde)

add_executable(contact_detection_example contact_detection_example.cpp)
target_link_libraries(contact_detection_example PUBLIC ur_rtde::rtde)