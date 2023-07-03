cmake_minimum_required(VERSION 3.0)
project(example)

find_package(dyn2b)
find_package(robif2b)

add_executable(main main.c)
target_link_libraries(main dyn2b::dyn2b robif2b::kinova_gen3)