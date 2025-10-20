cmake_minimum_required(VERSION 3.20)
project(DrawingShapeRecognition VERSION 1.0.0 LANGUAGES CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# Find OpenCV
find_package(OpenCV REQUIRED)

# Project information
message(STATUS "")
message(STATUS "========================================")
message(STATUS "  ${PROJECT_NAME} v${PROJECT_VERSION}")
message(STATUS "========================================")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV libs: ${OpenCV_LIBS}")
message(STATUS "========================================")
message(STATUS "")

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

# Source files (will be populated as we create classes)
set(SOURCES
    src/main.cpp
    # src/VideoCapture.cpp
    # src/PerspectiveTransformer.cpp
    # src/ShapeDetector.cpp
    # src/ShapeClassifier.cpp
    # src/OutputManager.cpp
)

# Main executable
add_executable(shape_recognition ${SOURCES})

# Link libraries
target_link_libraries(shape_recognition ${OpenCV_LIBS})

# Install targets
install(TARGETS shape_recognition DESTINATION bin)

# Print post-build message
add_custom_command(TARGET shape_recognition POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo ""
    COMMAND ${CMAKE_COMMAND} -E echo "=========================================="
    COMMAND ${CMAKE_COMMAND} -E echo "  Build completed successfully!"
    COMMAND ${CMAKE_COMMAND} -E echo "  Executable: $<TARGET_FILE:shape_recognition>"
    COMMAND ${CMAKE_COMMAND} -E echo "=========================================="
    COMMAND ${CMAKE_COMMAND} -E echo ""
)

# Test executables
add_executable(test_aruco_detection test/test_aruco_detection.cpp)
target_link_libraries(test_aruco_detection ${OpenCV_LIBS})

# Bird's-eye view calibration verification test
add_executable(test_birdseye_calibration test/test_birdseye_calibration.cpp)
target_link_libraries(test_birdseye_calibration ${OpenCV_LIBS})

# Interactive calibration adjuster
add_executable(test_interactive_calibration test/test_interactive_calibration.cpp)
target_link_libraries(test_interactive_calibration ${OpenCV_LIBS})

# Calibration tool
add_executable(camera_calibration tools/2_calibrate_camera.cpp)
target_link_libraries(camera_calibration ${OpenCV_LIBS})