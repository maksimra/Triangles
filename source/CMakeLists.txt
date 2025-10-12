add_executable (Triangles
                mathUtils.cpp
                main.cpp
)

target_link_libraries (Triangles PRIVATE triangles_lib)

target_include_directories (Triangles PUBLIC ${PROJECT_SOURCE_DIR}
                                          ${PROJECT_SOURCE_DIR}/include)