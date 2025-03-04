cmake_minimum_required(VERSION 3.8)
project(path_searching)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(plan_env)

add_library(${PROJECT_NAME} SHARED
        src/dyn_a_star.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(
        ${PROJECT_NAME} PUBLIC
        plan_env
)

install(TARGETS path_searching
        DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

ament_export_libraries(path_searching)
ament_export_dependencies(${dependencies})

ament_package()
