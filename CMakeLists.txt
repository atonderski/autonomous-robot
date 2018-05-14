cmake_minimum_required(VERSION 3.2)
project(autonomous-robot)

################################################################################
# Defining the relevant versions of OpenDLV Standard Message Set and libcluon.
set(OPENDLV_STANDARD_MESSAGE_SET opendlv-standard-message-set-v0.9.4.odvd)
set(CLUON_COMPLETE cluon-complete-v0.0.74.hpp)

################################################################################
# This project requires C++14 or newer.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # NEW!
set(CMAKE_CXX_EXTENSIONS OFF) # NEW!
# Strip unneeded symbols from binaries.
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -s") # NEW!
# Build a static binary.
set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++") # NEW!
# Add further warning levels.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
    -D_XOPEN_SOURCE=700 \
    -D_FORTIFY_SOURCE=2 \
    -O2 \
    -fstack-protector \
    -fomit-frame-pointer \
    -pipe \
    -pedantic -pedantic-errors \
    -Werror \
    -Weffc++ \
    -Wall -Wextra -Wshadow -Wdeprecated \
    -Wdiv-by-zero -Wfloat-equal -Wfloat-conversion -Wsign-compare -Wpointer-arith \
    -Wuninitialized -Wunreachable-code \
    -Wunused -Wunused-function -Wunused-label -Wunused-parameter -Wunused-but-set-parameter -Wunused-but-set-variable \
    -Wunused-value -Wunused-variable -Wunused-result \
    -Wmissing-field-initializers -Wmissing-format-attribute -Wmissing-include-dirs -Wmissing-noreturn")


# Threads are necessary for linking the resulting binaries as UDPReceiver is running in parallel.
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Is this one really necessary? # Yes I need it for vim autocomplete

################################################################################
# Extract cluon-msc from cluon-complete.hpp.
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/cluon-msc
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/src/${CLUON_COMPLETE} ${CMAKE_BINARY_DIR}/cluon-complete.hpp
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_BINARY_DIR}/cluon-complete.hpp ${CMAKE_BINARY_DIR}/cluon-complete.cpp
    COMMAND ${CMAKE_CXX_COMPILER} -o ${CMAKE_BINARY_DIR}/cluon-msc ${CMAKE_BINARY_DIR}/cluon-complete.cpp -std=c++14 -pthread -D HAVE_CLUON_MSC
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/${CLUON_COMPLETE})

################################################################################
# Generate opendlv-standard-message-set.{hpp,cpp} from ${OPENDLV_STANDARD_MESSAGE_SET} file.
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/opendlv-standard-message-set.cpp
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_BINARY_DIR}/cluon-msc --cpp-sources --cpp-add-include-file=opendlv-standard-message-set.hpp --out=${CMAKE_BINARY_DIR}/opendlv-standard-message-set.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/${OPENDLV_STANDARD_MESSAGE_SET}
    COMMAND ${CMAKE_BINARY_DIR}/cluon-msc --cpp-headers --out=${CMAKE_BINARY_DIR}/opendlv-standard-message-set.hpp ${CMAKE_CURRENT_SOURCE_DIR}/src/${OPENDLV_STANDARD_MESSAGE_SET}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/${OPENDLV_STANDARD_MESSAGE_SET} ${CMAKE_BINARY_DIR}/cluon-msc)

################################################################################
# Add current build directory as include directory as it contains generated files.
include_directories(SYSTEM ${CMAKE_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

################################################################################
# Gather all object code first to avoid double compilation.
add_library(${PROJECT_NAME}-core OBJECT ${CMAKE_BINARY_DIR}/opendlv-standard-message-set.cpp src/manualcontroller.cpp src/subsumer.cpp src/pathfinder.cpp src/path-maker.cpp src/behaviours/behaviour-reflex.cpp src/behaviours/behaviour-motivation.cpp src/behaviours/behaviour-avoid.cpp src/behaviours/behaviour-followpath.cpp)
set(LIBRARIES Threads::Threads)

################################################################################
# Create executable.
add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}.cpp $<TARGET_OBJECTS:${PROJECT_NAME}-core>)
target_link_libraries(${PROJECT_NAME} ${LIBRARIES})

################################################################################
# Enable unit testing.
# enable_testing()
# add_executable(${PROJECT_NAME}-runner ${CMAKE_CURRENT_SOURCE_DIR}/tests/test-prime-checker.cpp $<TARGET_OBJECTS:${PROJECT_NAME}-core>)
# target_link_libraries(${PROJECT_NAME}-runner ${LIBRARIES})
# add_test(NAME ${PROJECT_NAME}-runner COMMAND ${PROJECT_NAME}-runner)

################################################################################
# Install executable.
# install(TARGETS ${PROJECT_NAME} DESTINATION bin COMPONENT ${PROJECT_NAME})