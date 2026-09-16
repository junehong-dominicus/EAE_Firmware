cmake_minimum_required(VERSION 3.16)
project(EAE_Firmware CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra)
    # Statically link the C++/pthread runtime so the built binaries have no
    # external runtime DLL/.so to ship alongside them.
    add_link_options(-static)
endif()

add_library(cooling_core
    src/can_bus.cpp
    src/can_codec.cpp
    src/pid_controller.cpp
    src/cooling_state_machine.cpp
)
target_include_directories(cooling_core PUBLIC include)

add_executable(eae_firmware src/main.cpp)
target_link_libraries(eae_firmware PRIVATE cooling_core)

option(EAE_BUILD_TESTS "Build the GTest unit tests" ON)
if (EAE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
