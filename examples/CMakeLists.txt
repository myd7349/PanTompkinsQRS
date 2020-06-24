if(NOT DEFINED PROJECT_NAME)
    cmake_minimum_required(VERSION 3.8)

    project(example C)
    
    find_package(pantompkinsqrs CONFIG REQUIRED)
endif()

add_subdirectory(importdata EXCLUDE_FROM_ALL)

add_executable(example example.c)
if(MSVC)
    target_compile_definitions(example PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
target_link_libraries(example PRIVATE importdata::importdata pantompkinsqrs::pantompkinsqrs)

add_custom_command(
    TARGET example
    POST_BUILD
    COMMAND "${CMAKE_COMMAND}"
    ARGS -E copy_if_different "${CMAKE_CURRENT_LIST_DIR}/test_input.txt" $<TARGET_FILE_DIR:example>
    WORKING_DIRECTORY "${PROJECT_BINARY_DIR}"
    VERBATIM
)
