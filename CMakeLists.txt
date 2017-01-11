cmake_minimum_required(VERSION 3.1)

add_compile_options(-std=c++11)

project(SerialSimulator)
add_executable(${PROJECT_NAME} "main.cpp")

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} Threads::Threads)
