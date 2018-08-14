set(CATCH_INCLUDE_DIR third_party)
set(TEST_PROJECT_NAME ${PROJECT_NAME}Test)

add_library(Catch INTERFACE)
target_include_directories(Catch INTERFACE ${CATCH_INCLUDE_DIR})

file(GLOB TEST_SOURCES "*.cpp" "ik/*.cpp")

add_executable(${TEST_PROJECT_NAME} ${TEST_SOURCES})
target_link_libraries(${TEST_PROJECT_NAME} RobotLib Catch)
