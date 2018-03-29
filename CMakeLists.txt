cmake_minimum_required(VERSION 3.2)
project(autonomous-robot)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Threads REQUIRED)

################################################################################
# Generate opendlv-standard-message-set.{hpp,cpp} from ${OPENDLV_STANDARD_MESSAGE_SET} file.
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/opendlv-standard-message-set.cpp
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND cluon-msc --cpp-sources --cpp-add-include-file=opendlv-standard-message-set.hpp --out=${CMAKE_BINARY_DIR}/opendlv-standard-message-set.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/opendlv.odvd
    COMMAND cluon-msc --cpp-headers --out=${CMAKE_BINARY_DIR}/opendlv-standard-message-set.hpp ${CMAKE_CURRENT_SOURCE_DIR}/src/opendlv.odvd
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/opendlv.odvd)

################################################################################
# Generate messages.{hpp,cpp} from messages file.
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/messages.cpp
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND cluon-msc --cpp-sources --cpp-add-include-file=messages.hpp --out=${CMAKE_BINARY_DIR}/messages.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/messages.odvd
    COMMAND cluon-msc --cpp-headers --out=${CMAKE_BINARY_DIR}/messages.hpp ${CMAKE_CURRENT_SOURCE_DIR}/src/messages.odvd
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/messages.odvd)

# Add current build directory as include directory as it contains generated files.
include_directories(SYSTEM ${CMAKE_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

################################################################################
# Gather all object code first to avoid double compilation.
add_library(${PROJECT_NAME}-core OBJECT  ${CMAKE_BINARY_DIR}/messages.cpp ${CMAKE_BINARY_DIR}/opendlv-standard-message-set.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/prime-checker.cpp)
set(LIBRARIES Threads::Threads)

################################################################################
# Create executable.
add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}.cpp $<TARGET_OBJECTS:${PROJECT_NAME}-core>)
target_link_libraries(${PROJECT_NAME} ${LIBRARIES})

add_executable(helloworld ${CMAKE_CURRENT_SOURCE_DIR}/src/helloworld.cpp $<TARGET_OBJECTS:${PROJECT_NAME}-core>)
target_link_libraries(helloworld ${LIBRARIES})

################################################################################
# Enable unit testing.
enable_testing()
add_executable(${PROJECT_NAME}-runner ${CMAKE_CURRENT_SOURCE_DIR}/test/test-prime-checker.cpp $<TARGET_OBJECTS:${PROJECT_NAME}-core>)
add_test(NAME ${PROJECT_NAME}-runner COMMAND ${PROJECT_NAME}-runner)
