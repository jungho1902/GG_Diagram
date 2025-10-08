include(FetchContent)

set(IMGUI_VERSION v1.90.9)
FetchContent_Declare(
    dear_imgui
    URL https://github.com/ocornut/imgui/archive/refs/tags/${IMGUI_VERSION}.zip
)

FetchContent_GetProperties(dear_imgui)
if (NOT dear_imgui_POPULATED)
    FetchContent_Populate(dear_imgui)
endif()

set(IMGUI_SOURCE_FILES
    ${dear_imgui_SOURCE_DIR}/imgui.cpp
    ${dear_imgui_SOURCE_DIR}/imgui_demo.cpp
    ${dear_imgui_SOURCE_DIR}/imgui_draw.cpp
    ${dear_imgui_SOURCE_DIR}/imgui_tables.cpp
    ${dear_imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${dear_imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${dear_imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

add_library(imgui STATIC ${IMGUI_SOURCE_FILES})

set_target_properties(imgui PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(imgui
    PUBLIC
        ${dear_imgui_SOURCE_DIR}
        ${dear_imgui_SOURCE_DIR}/backends
)

target_compile_definitions(imgui PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLEW)

target_sources(imgui PRIVATE ${dear_imgui_SOURCE_DIR}/misc/cpp/imgui_stdlib.cpp)

