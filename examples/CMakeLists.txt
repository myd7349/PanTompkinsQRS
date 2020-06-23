add_executable(example example.c)
target_include_directories(example PRIVATE ${PAN_TOMPKINS_QRS_INCLUDE_DIR})
target_link_libraries(example PRIVATE pantompkinsqrs)
