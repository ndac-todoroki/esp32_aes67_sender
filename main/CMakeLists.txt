message(STATUS "######################################################################################")
message(STATUS "CMAKE_SOURCE_DIR is '${CMAKE_SOURCE_DIR}'.")
message(STATUS "CMAKE_BINARY_DIR is '${CMAKE_BINARY_DIR}'.")
message(STATUS "CMAKE_CURRENT_SOURCE_DIR is '${CMAKE_CURRENT_SOURCE_DIR}'.")
message(STATUS "CMAKE_CURRENT_BINARY_DIR is '${CMAKE_CURRENT_BINARY_DIR}'.")
message(STATUS "PROJECT_SOURCE_DIR is '${PROJECT_SOURCE_DIR}'.")
message(STATUS "PROJECT_BINARY_DIR is '${PROJECT_BINARY_DIR}'.")
message(STATUS "aes67_sender_SOURCE_DIR is '${aes67_sender_SOURCE_DIR}'.")
message(STATUS "aes67_sender_BINARY_DIR is '${aes67_sender_BINARY_DIR}'.")
message(STATUS "CMAKE_CURRENT_LIST_DIR is '${CMAKE_CURRENT_LIST_DIR}'.")
message(STATUS "Toolchain is '${TOOLCHAIN}'.")
message(STATUS "######################################################################################")

# Include config
include(${CMAKE_CURRENT_LIST_DIR}/config/cmake_config.txt)

# Enable color even if using Ninja.
option (FORCE_COLORED_OUTPUT "Always produce ANSI-colored output (GNU/Clang only)." TRUE)
if (${FORCE_COLORED_OUTPUT})
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
       add_compile_options (-fdiagnostics-color=always)
    elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
       add_compile_options (-fcolor-diagnostics)
    endif ()
endif ()

set(COMPONENT_SRCS ${MainFile})
set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()
