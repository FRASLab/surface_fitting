cmake_minimum_required(VERSION 3.0.2)
project(surface_contact)

add_definitions(-DPROJECT_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

find_package(PCL 1.14 REQUIRED COMPONENTS
    common
    surface
    filters
    io)

find_package(VTK REQUIRED)

include_directories(
        include
        ${PCL_INCLUDE_DIRS}
        ${FLANN_INCLUDE_DIRS}
        ${VTK_INCLUDE_DIRS})

add_library(surface_contact SHARED src/nurbs.cpp src/invkin.cpp)
target_link_libraries(surface_contact
        ${PCL_COMMON_LIBRARIES}
        ${PCL_SURFACE_LIBRARIES}
        ${PCL_IO_LIBRARIES}
        ${VTK_LIBRARIES})

add_executable(test_demo src/test_demo.cpp)
add_dependencies(test_demo surface_contact)
target_link_libraries(test_demo
        ${PCL_COMMON_LIBRARIES}
        ${PCL_SURFACE_LIBRARIES}
        ${PCL_IO_LIBRARIES}
        ${VTK_LIBRARIES}
        surface_contact)

set(INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/surface_contact)

configure_file(surface_contactConfig.cmake.in
        surface_contactConfig.cmake @ONLY)

install(TARGETS surface_contact
        RUNTIME DESTINATION ${INSTALL_DIR}/bin
        LIBRARY DESTINATION ${INSTALL_DIR}/lib
        ARCHIVE DESTINATION ${INSTALL_DIR}/lib)

install(DIRECTORY include
        DESTINATION ${INSTALL_DIR})

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/surface_contactConfig.cmake
        DESTINATION ${INSTALL_DIR}/cmake)



