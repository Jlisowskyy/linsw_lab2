cmake_minimum_required(VERSION 3.30)
project(linsw C)

set(CMAKE_C_STANDARD 17)

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(PERIPHERY periphery)
endif()

if(NOT PERIPHERY_FOUND)
    message(STATUS "c-periphery not found on system, fetching from source")
    include(FetchContent)

    FetchContent_Declare(
            periphery
            GIT_REPOSITORY https://github.com/vsergeev/c-periphery.git
            GIT_TAG master
    )

    FetchContent_MakeAvailable(periphery)

    set(PERIPHERY_LIBRARIES periphery)
    set(PERIPHERY_INCLUDE_DIRS "${periphery_SOURCE_DIR}")
else()
    message(STATUS "Using system-installed c-periphery")
endif()

add_executable(linsw main.c)

target_include_directories(linsw PRIVATE ${PERIPHERY_INCLUDE_DIRS})

target_link_libraries(linsw ${PERIPHERY_LIBRARIES})

install(TARGETS linsw DESTINATION bin)
