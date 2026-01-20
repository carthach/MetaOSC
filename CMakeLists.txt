cmake_minimum_required(VERSION 3.22)

project(MetaOSC VERSION 1.0.0 LANGUAGES CXX)

# Include FetchContent for downloading external dependencies
include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
FetchContent_MakeAvailable(json)

# Fetch MetaWear SDK from GitHub
FetchContent_Declare(
    MetaWearSDK
    GIT_REPOSITORY https://github.com/mbientlab/MetaWear-SDK-Cpp.git
    GIT_TAG master
)

# Fetch SimpleBLE from GitHub
FetchContent_Declare(
    SimpleBLE
    GIT_REPOSITORY https://github.com/simpleble/simpleble.git
    GIT_TAG v0.6.1
)

# Make MetaWear SDK available
FetchContent_MakeAvailable(MetaWearSDK)

# Make SimpleBLE available
FetchContent_MakeAvailable(SimpleBLE)

# Add SimpleBLE subdirectory
add_subdirectory(${simpleble_SOURCE_DIR}/simpleble ${simpleble_BINARY_DIR})

# Override MetaWear SDK compiler flags to remove -Werror (treat warnings as errors)
# This prevents build failures due to deprecated function warnings
target_compile_options(metawear PRIVATE -Wno-error=deprecated-declarations)

# Add JUCE submodule
add_subdirectory(JUCE)

# Create the MetaOSC console application
juce_add_console_app(MetaOSC
    PRODUCT_NAME "MetaOSC"
)

# Add source files
target_sources(MetaOSC
    PRIVATE
        src/Main.cpp
        src/metamotionController.cpp
        src/metamotionController.h
        src/bleInterface.h
)

# Generate JuceHeader.h
juce_generate_juce_header(MetaOSC)

# Include directories for external libraries
target_include_directories(MetaOSC PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/build/_deps/metawearsdk-src/src
)

# Platform-specific library paths and linking
if(APPLE)
    # macOS libraries and frameworks
    target_link_libraries(MetaOSC PRIVATE
        metawear
        simpleble
        "-framework CoreBluetooth"
        "-framework Foundation"
        "-framework CoreFoundation"
    )
elseif(WIN32)
    # Windows libraries
    target_link_libraries(MetaOSC PRIVATE
        metawear
        simpleble
    )
endif()

# Set compile definitions
target_compile_definitions(MetaOSC PRIVATE
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
)

# Link JUCE modules
target_link_libraries(MetaOSC
    PRIVATE
        juce::juce_core
        juce::juce_events
        juce::juce_osc
        juce::juce_data_structures
        nlohmann_json::nlohmann_json
    PUBLIC
        juce::juce_recommended_config_flags
        juce::juce_recommended_warning_flags
)

