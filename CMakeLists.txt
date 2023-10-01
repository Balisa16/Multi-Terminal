cmake_minimum_required(VERSION 3.1)

project(multi-terminal)

find_package(PkgConfig REQUIRED)
find_package(jsoncpp REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
pkg_check_modules(VTE REQUIRED vte-2.91)


set(CMAKE_CXX_STANDARD 11)

add_executable(main src/main.cpp)

target_include_directories(main PRIVATE 
	${GTK3_INCLUDE_DIRS}
	${VTE_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/include 
	${jsoncpp_INCLUDE_DIRS})

target_link_libraries(main ${GTK3_LIBRARIES} ${VTE_LIBRARIES} jsoncpp)

target_compile_options(main PRIVATE ${GTK3_CFLAGS_OTHER} ${VTE_CFLAGS_OTHER})