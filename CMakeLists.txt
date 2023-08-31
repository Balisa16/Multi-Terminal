cmake_minimum_required(VERSION 3.1)

project(Multiple-Terminal)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
pkg_check_modules(VTE REQUIRED vte-2.91)

set(CMAKE_CXX_STANDARD 11)

# include_directories(
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
#   ${GTK3_INCLUDE_DIRS}
#   ${VTE_INCLUDE_DIRS})

add_executable(main1 src/main1.cpp)
add_executable(main2 src/main2.cpp)
add_executable(main3 src/main3.cpp)

target_include_directories(main1 PRIVATE ${GTK3_INCLUDE_DIRS} ${VTE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(main2 PRIVATE ${GTK3_INCLUDE_DIRS} ${VTE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(main3 PRIVATE ${GTK3_INCLUDE_DIRS} ${VTE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(main1 ${GTK3_LIBRARIES} ${VTE_LIBRARIES})
target_link_libraries(main2 ${GTK3_LIBRARIES} ${VTE_LIBRARIES})
target_link_libraries(main3 ${GTK3_LIBRARIES} ${VTE_LIBRARIES})

# Add compile options for GTK and VTE
target_compile_options(main1 PRIVATE ${GTK3_CFLAGS_OTHER} ${VTE_CFLAGS_OTHER})
target_compile_options(main2 PRIVATE ${GTK3_CFLAGS_OTHER} ${VTE_CFLAGS_OTHER})
target_compile_options(main3 PRIVATE ${GTK3_CFLAGS_OTHER} ${VTE_CFLAGS_OTHER})