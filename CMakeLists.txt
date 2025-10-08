cmake_minimum_required(VERSION 3.20)
project(GGDiagramGUI LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(GLEW REQUIRED)

add_subdirectory(vendor)

add_library(gg_core STATIC
    src/analytics.cpp
    src/attitude_estimator.cpp
    src/gg_envelope.cpp
    src/gg_processor.cpp
    src/quasi_steady_state_filter.cpp
)

target_include_directories(gg_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(gg_core PUBLIC cxx_std_17)

add_executable(gg_gui
    src/main.cpp
    gui/GuiApplication.cpp
    gui/SimulationPipeline.cpp
)

target_include_directories(gg_gui
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/gui
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(gg_gui
    PRIVATE
        gg_core
        imgui
        glfw
        OpenGL::GL
        GLEW::GLEW
)

if (MSVC)
    target_compile_options(gg_core PRIVATE /W4 /permissive-)
    target_compile_options(gg_gui PRIVATE /W4 /permissive-)
else()
    target_compile_options(gg_core PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(gg_gui PRIVATE -Wall -Wextra -Wpedantic)
endif()
