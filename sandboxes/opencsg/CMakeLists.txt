cmake_minimum_required(VERSION 3.0)

project(OpenCSG-example)

add_executable(opencsg_example main.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/slic3r/GUI/3DEngine.cpp
    )

set (wxWidgets_USE_STATIC ON )
set (wxWidgets_CONFIG_OPTIONS "--toolkit=gtk${SLIC3R_GTK}")
find_package(wxWidgets 3.1 REQUIRED COMPONENTS core base gl)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
include(${wxWidgets_USE_FILE})


target_link_libraries(opencsg_example libslic3r)
target_include_directories(opencsg_example PRIVATE ${wxWidgets_INCLUDE_DIRS})
target_compile_definitions(opencsg_example PRIVATE ${wxWidgets_DEFINITIONS})
target_link_libraries(opencsg_example ${wxWidgets_LIBRARIES} GLEW::GLEW OpenGL::OpenGL)
