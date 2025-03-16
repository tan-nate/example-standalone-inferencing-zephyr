cmake_minimum_required(VERSION 3.13.1)

# Set project name
project(example-standalone-inferencing-zephyr)

# Ensure correct C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Zephyr SDK
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Enable colored output for debugging
zephyr_compile_options(-fdiagnostics-color=always)

# Enable hardware acceleration for DSP and ML inference
add_definitions(
    -DEI_CLASSIFIER_TFLITE_ENABLE_CMSIS_NN=1
    -DARM_MATH_LOOPUNROLL
)

# Define source directories
set(SOURCE_DIRS
    src
    tflite-model
    model-parameters
)

# Add include directories
include_directories(${SOURCE_DIRS})

# Add Edge Impulse SDK (Ensure correct path)
add_subdirectory(edge-impulse-sdk)

# Collect all .cpp source files in tflite-model directory
file(GLOB_RECURSE MODEL_FILES "tflite-model/*.cpp")

# Define the list of source files
set(SOURCE_FILES
    ${MODEL_FILES}
    src/main.cpp
    src/adbms_main.cpp
    src/adBms_Application.cpp
)

# Add sources to the Zephyr application
target_sources(app PRIVATE ${SOURCE_FILES})
