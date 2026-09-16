include(FetchContent)

# GTest is the only external dependency, and it is fetched by the build
# system at configure time rather than vendored/shipped in this repo.
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
)

# Match the parent project's runtime linking (relevant on MSVC; harmless on GCC/Clang).
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(eae_firmware_tests
    test_can_bus.cpp
    test_can_codec.cpp
    test_pid_controller.cpp
    test_cooling_state_machine.cpp
)
target_link_libraries(eae_firmware_tests PRIVATE cooling_core GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(eae_firmware_tests)
