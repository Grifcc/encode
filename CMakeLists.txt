cmake_minimum_required(VERSION 3.11.0)
project(rtsp_stream VERSION 1.0.0 LANGUAGES CXX C)

set(CXX_STD "14" CACHE STRING "C++ standard")

if(CMAKE_TOOLCHAIN_FILE)
  get_filename_component(CMAKE_TOOLCHAIN_FILE_NAME ${CMAKE_TOOLCHAIN_FILE} NAME)
  find_file(CMAKE_TOOLCHAIN_FILE ${CMAKE_TOOLCHAIN_FILE_NAME} PATHS ${CMAKE_SOURCE_DIR} NO_DEFAULT_PATH)
  message(STATUS "CMAKE_TOOLCHAIN_FILE = ${CMAKE_TOOLCHAIN_FILE}")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--allow-shlib-undefined  -Wno-class-memaccess -Wno-deprecated-declarations -Wno-sign-compare")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wl,--allow-shlib-undefined  -Wno-class-memaccess -Wno-deprecated-declarations -Wno-sign-compare")

set(CMAKE_CXX_STANDARD "${CXX_STD}")
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# BUILD  TYPE
if(CMAKE_BUILD_TYPE AND(CMAKE_BUILD_TYPE STREQUAL "Debug"))
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS} -Wall -O0  -g")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -O0  -g")
  message(STATUS "Debug mode:")
  message(STATUS "CMAKE_C_FLAGS_DEBUG: ${CMAKE_C_FLAGS_DEBUG}")
  message(STATUS "CMAKE_CXX_FLAGS_DEBUG: ${CMAKE_CXX_FLAGS_DEBUG}")
  add_definitions(-DDEBUG)
elseif(CMAKE_BUILD_TYPE AND(CMAKE_BUILD_TYPE STREQUAL "Release"))
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS} -Wall -O3")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Wall -O3")
  message(STATUS "Release mode:")
  message(STATUS "CMAKE_C_FLAGS_RELEASE: ${CMAKE_C_FLAGS_RELEASE}")
  message(STATUS "CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")
  add_definitions(-DNDEBUG)
else()
  message("The CMAKE_BUILD_TYPE is not specified, defaulting to Debug mode.")
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS} -Wall -O0  -g")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -O0  -g")
  message(STATUS "Debug mode:")
  message(STATUS "CMAKE_C_FLAGS_DEBUG: ${CMAKE_C_FLAGS_DEBUG}")
  message(STATUS "CMAKE_CXX_FLAGS_DEBUG: ${CMAKE_CXX_FLAGS_DEBUG}")
  add_definitions(-DDEBUG)
endif()

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/build/bin)

# install target and libraries
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)

set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")

set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_BUILD_RPATH "$ORIGIN/../lib")

set(COMMON_HEADER_DIRS
  ${PROJECT_SOURCE_DIR}/include
)

# opencv
find_package(OpenCV 4 REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})

# ffmpeg
include_directories(/usr/local/ffmpeg/include)
link_directories(/usr/local/ffmpeg/lib)
list(APPEND CMAKE_MODULE_PATH /usr/local/ffmpeg/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(
  ${COMMON_HEADER_DIRS}
)

# 添加动态链接库
set(dynamic_libs
  pthread
)

message(STATUS "dynamic_libs :${dynamic_libs}")
aux_source_directory(${PROJECT_SOURCE_DIR}/src/ DET_SRC_DIR)

set(CMAKE_INSTALL_RPATH "lib")

# install target and libraries
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)
message(STATUS "CMAKE_INSTALL_PREFIX :${CMAKE_INSTALL_PREFIX}")

add_library(rtsp_stream SHARED src/rtsp_stream.cpp)
target_link_libraries(rtsp_stream
  avformat
  avfilter
  avcodec
  swscale
  swresample
  avutil
  avdevice
)

add_executable(main main.cpp)
target_link_libraries(main
  rtsp_stream
  ${OpenCV_LIBS}
)

install(TARGETS main DESTINATION ./)
install(TARGETS rtsp_stream DESTINATION ./lib)
install(FILES ${PROJECT_SOURCE_DIR}/include/rtsp_stream.h DESTINATION ./include/)
install(FILES ${PROJECT_SOURCE_DIR}/include/threadsafe_queue.h DESTINATION ./include/)
