# SPDX-FileCopyrightText: 2025, FANUC America Corporation
# SPDX-FileCopyrightText: 2025, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(fanuc_client)

option(BUILD_EXAMPLES "Build examples" OFF)
option(FANUC_CLIENT_COMMAND_PLOT "Build live ImGui/ImPlot pre-socket command plotter" ON)

# Find dependencies
find_package(Eigen3 REQUIRED)

set(FANUC_CLIENT_SOURCES
    src/fanuc_client.cpp
    src/gpio_buffer.cpp
    src/joint_stream_interpolator.cpp
    src/rmi_singleton.cpp
)

if(FANUC_CLIENT_COMMAND_PLOT)
  list(APPEND FANUC_CLIENT_SOURCES src/joint_command_plotter.cpp)
endif()

add_library(fanuc_client SHARED ${FANUC_CLIENT_SOURCES})
set_target_properties(fanuc_client PROPERTIES CXX_STANDARD 17)
target_link_libraries(fanuc_client PUBLIC stream_motion rmi Eigen3::Eigen)
target_link_libraries(fanuc_client PRIVATE readerwriterqueue)
target_include_directories(fanuc_client PUBLIC
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(FANUC_CLIENT_COMMAND_PLOT)
  include(FetchContent)

  find_package(OpenGL REQUIRED)

  # Static GLFW so libfanuc_client.so does not need libglfw.so at runtime.
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  set(GLFW_LIBRARY_TYPE "STATIC" CACHE STRING "" FORCE)
  FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
  )
  FetchContent_MakeAvailable(glfw)

  FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.8
  )
  FetchContent_GetProperties(imgui)
  if(NOT imgui_POPULATED)
    FetchContent_Populate(imgui)
  endif()

  FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG v0.16
  )
  FetchContent_GetProperties(implot)
  if(NOT implot_POPULATED)
    FetchContent_Populate(implot)
  endif()

  add_library(fanuc_imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
  )
  set_target_properties(fanuc_imgui PROPERTIES
    CXX_STANDARD 17
    POSITION_INDEPENDENT_CODE ON
  )
  target_include_directories(fanuc_imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${implot_SOURCE_DIR}
  )
  target_link_libraries(fanuc_imgui PUBLIC glfw OpenGL::GL)
  if(UNIX AND NOT APPLE)
    target_link_libraries(fanuc_imgui PUBLIC dl pthread)
  endif()

  target_compile_definitions(fanuc_client PUBLIC FANUC_CLIENT_COMMAND_PLOT=1)
  target_link_libraries(fanuc_client PRIVATE fanuc_imgui)
endif()

if (BUILD_TESTING)
  add_subdirectory(test)
endif ()
