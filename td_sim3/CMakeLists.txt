cmake_minimum_required(VERSION 3.20)
project(td_sim3 CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# --- Dependencies -----------------------------------------------------------
# Boost (for odeint, header-only). Find system Boost first.
find_package(Boost REQUIRED)

# spdlog (structured logging for debug/diagnostics output).
find_package(spdlog REQUIRED)

# toml++ (user-facing vehicle config files, parsed strictly at load time).
find_package(tomlplusplus REQUIRED)

# --- Core library -----------------------------------------------------------
add_library(tdsim STATIC
  src/topology.cpp
  src/telemetry.cpp
  src/simulator.cpp
  src/factory.cpp
  src/registry_builtin.cpp
  src/registry_flight.cpp
  src/config.cpp
)
target_include_directories(tdsim PUBLIC include)
target_link_libraries(tdsim PUBLIC Boost::boost spdlog::spdlog tomlplusplus::tomlplusplus)
target_compile_options(tdsim PRIVATE -Wall -Wextra)

# --- Example ----------------------------------------------------------------
# Reference vehicle configs live with the executable that consumes them;
# the tdsim core library itself has no knowledge of any specific vehicle.
add_executable(td_sim_example examples/main.cpp)
target_link_libraries(td_sim_example PRIVATE tdsim Boost::boost spdlog::spdlog)
add_executable(td_sim examples/td_sim.cpp)
target_link_libraries(td_sim PRIVATE tdsim Boost::boost spdlog::spdlog tomlplusplus::tomlplusplus)

# Copy the reference config next to the binaries for convenience.
add_custom_command(TARGET td_sim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/config/vehicle.toml
        ${CMAKE_CURRENT_BINARY_DIR}/vehicle.toml
    COMMENT "Copying reference vehicle config")

# --- Tests (doctest) --------------------------------------------------------
# doctest is header-only; the system install lives at /usr/include/doctest.
find_path(DOCTEST_INCLUDE_DIR doctest/doctest.h REQUIRED)

enable_testing()
add_executable(tdsim_tests
  tests/main.cpp
  tests/test_signals.cpp
  tests/test_topology.cpp
  tests/test_factory.cpp
  tests/test_multirate.cpp
)
target_include_directories(tdsim_tests PRIVATE ${DOCTEST_INCLUDE_DIR})
target_link_libraries(tdsim_tests PRIVATE tdsim)
add_test(NAME tdsim_tests COMMAND tdsim_tests)
