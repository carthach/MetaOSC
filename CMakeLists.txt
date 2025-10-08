cmake_minimum_required(VERSION 3.22)

project(MetaOSC VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# Avoid warnings-as-errors for external dependencies
if(POLICY CMP0092)
    cmake_policy(SET CMP0092 NEW)
    set(CMAKE_POLICY_DEFAULT_CMP0092 NEW)
endif()

# Fetch MetaWear SDK from GitHub
FetchContent_Declare(
    MetaWearSDK
    GIT_REPOSITORY https://github.com/mbientlab/MetaWear-SDK-Cpp.git
    GIT_TAG master
    PATCH_COMMAND ""
)

# Fetch SimpleBLE from GitHub
FetchContent_Declare(
    SimpleBLE
    GIT_REPOSITORY https://github.com/simpleble/simpleble.git
    GIT_TAG v0.6.1
)

# Patch and configure MetaWear SDK for cross-platform compatibility
FetchContent_GetProperties(MetaWearSDK)
if(NOT metawearsdk_POPULATED)
    FetchContent_Populate(MetaWearSDK)
    
    # Patch CMakeLists.txt to remove -Werror and fix MSVC compatibility
    set(METAWEAR_CMAKELISTS "${metawearsdk_SOURCE_DIR}/CMakeLists.txt")
    file(READ "${METAWEAR_CMAKELISTS}" METAWEAR_CMAKE_CONTENT)
    string(FIND "${METAWEAR_CMAKE_CONTENT}" "# PATCHED FOR CROSS-PLATFORM" ALREADY_PATCHED)
    
    if(ALREADY_PATCHED EQUAL -1)
        string(REPLACE 
            "set(COMMON_FLAGS \"-std=c++11 -fPIC -fvisibility=hidden -fvisibility-inlines-hidden -Wall -Werror -I\${SOURCE_DIR} -DMETAWEAR_DLL -DMETAWEAR_DLL_EXPORTS \${OPT_FLAGS}\")"
            "# PATCHED FOR CROSS-PLATFORM\nif(NOT MSVC)\n    set(COMMON_FLAGS \"-std=c++11 -fPIC -fvisibility=hidden -fvisibility-inlines-hidden -Wall -I\${SOURCE_DIR} -DMETAWEAR_DLL -DMETAWEAR_DLL_EXPORTS \${OPT_FLAGS}\")\nelse()\n    set(COMMON_FLAGS \"\")\nendif()"
            METAWEAR_CMAKE_CONTENT "${METAWEAR_CMAKE_CONTENT}")
        file(WRITE "${METAWEAR_CMAKELISTS}" "${METAWEAR_CMAKE_CONTENT}")
        message(STATUS "Patched MetaWear SDK CMakeLists.txt")
    endif()
    
    if(MSVC)
        # Patch file_operations.cpp to fix ambiguous operator= error
        set(FILE_OPS_CPP "${metawearsdk_SOURCE_DIR}/src/metawear/dfu/cpp/file_operations.cpp")
        file(READ "${FILE_OPS_CPP}" FILE_OPS_CONTENT)
        string(FIND "${FILE_OPS_CONTENT}" "get<std::string>()" FILE_OPS_PATCHED)
        
        if(FILE_OPS_PATCHED EQUAL -1)
            string(REPLACE "firmwareFilename = *it2;" "firmwareFilename = it2->get<std::string>();"
                FILE_OPS_CONTENT "${FILE_OPS_CONTENT}")
            string(REPLACE "metadataFilename = *it3;" "metadataFilename = it3->get<std::string>();"
                FILE_OPS_CONTENT "${FILE_OPS_CONTENT}")
            file(WRITE "${FILE_OPS_CPP}" "${FILE_OPS_CONTENT}")
            message(STATUS "Patched file_operations.cpp for MSVC")
        endif()
        
        # Clear CMAKE_CXX_FLAGS for MetaWear subdirectory on MSVC
        set(SAVED_CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
        set(CMAKE_CXX_FLAGS "")
        add_subdirectory(${metawearsdk_SOURCE_DIR} ${metawearsdk_BINARY_DIR} EXCLUDE_FROM_ALL)
        set(CMAKE_CXX_FLAGS "${SAVED_CMAKE_CXX_FLAGS}")
    else()
        add_subdirectory(${metawearsdk_SOURCE_DIR} ${metawearsdk_BINARY_DIR})
    endif()
endif()

# Make SimpleBLE available
FetchContent_MakeAvailable(SimpleBLE)

# Configure MetaWear target compiler flags
if(TARGET metawear)
    if(MSVC)
        set_target_properties(metawear PROPERTIES COMPILE_OPTIONS "" COMPILE_FLAGS "")
        target_compile_options(metawear PRIVATE /W3 /DMETAWEAR_DLL /DMETAWEAR_DLL_EXPORTS)
        
        get_target_property(METAWEAR_INTERFACE_COMPILE_OPTIONS metawear INTERFACE_COMPILE_OPTIONS)
        if(METAWEAR_INTERFACE_COMPILE_OPTIONS)
            set_target_properties(metawear PROPERTIES INTERFACE_COMPILE_OPTIONS "")
        endif()
    else()
        target_compile_options(metawear PRIVATE 
            -Wno-error=deprecated-declarations
            -Wno-error=maybe-uninitialized)
    endif()
endif()

# Add JUCE framework
add_subdirectory(JUCE)

# Create MetaOSC console application
juce_add_console_app(MetaOSC PRODUCT_NAME "MetaOSC" VERSION "1.0.0")

target_sources(MetaOSC PRIVATE
    src/Main.cpp
    src/MetaMotionController.cpp
    src/MetaMotionController.h
    src/BleInterface.h)

juce_generate_juce_header(MetaOSC)

target_include_directories(MetaOSC PRIVATE
    ${metawearsdk_SOURCE_DIR}/src
    ${simpleble_SOURCE_DIR}/simpleble/include)

# Link external libraries
target_link_libraries(MetaOSC PRIVATE metawear simpleble)

# Platform-specific frameworks (macOS)
if(APPLE)
    target_link_libraries(MetaOSC PRIVATE
        "-framework CoreBluetooth"
        "-framework Foundation"
        "-framework CoreFoundation")
endif()

# JUCE compile definitions
target_compile_definitions(MetaOSC PRIVATE
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0)

# Link JUCE modules
target_link_libraries(MetaOSC PRIVATE
    juce::juce_core
    juce::juce_events
    juce::juce_osc
    PUBLIC
    juce::juce_recommended_config_flags
    juce::juce_recommended_warning_flags)
