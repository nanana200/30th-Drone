cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME gpslora222)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

set(CUBEMX_GENERATED_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32h7xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32h7xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/system_stm32h7xx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32h753xx.s
)

file(GLOB_RECURSE AUTO_DISCOVERED_C_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/*.c
)

list(FILTER AUTO_DISCOVERED_C_SOURCES EXCLUDE REGEX "/(build|cmake|Drivers|OLED FILE)/")
list(REMOVE_ITEM AUTO_DISCOVERED_C_SOURCES ${CUBEMX_GENERATED_SOURCES})

file(GLOB_RECURSE AUTO_DISCOVERED_HEADERS CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/*.h
)

set(AUTO_DISCOVERED_INCLUDE_DIRS)
foreach(AUTO_DISCOVERED_HEADER ${AUTO_DISCOVERED_HEADERS})
    if(NOT AUTO_DISCOVERED_HEADER MATCHES "/(build|cmake|Drivers|OLED FILE)/")
        get_filename_component(AUTO_DISCOVERED_INCLUDE_DIR ${AUTO_DISCOVERED_HEADER} DIRECTORY)
        list(APPEND AUTO_DISCOVERED_INCLUDE_DIRS ${AUTO_DISCOVERED_INCLUDE_DIR})
    endif()
endforeach()

list(REMOVE_DUPLICATES AUTO_DISCOVERED_INCLUDE_DIRS)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${AUTO_DISCOVERED_C_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${AUTO_DISCOVERED_INCLUDE_DIRS}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
