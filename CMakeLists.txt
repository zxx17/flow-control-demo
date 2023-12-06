# CMakeList.txt: CMakeProject1 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)

project ("CMakeProject1")
# 库目录
set(CMAKE_CXX_STANDARD 17)
set(CS20_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/res/sdk/lib")
set(CS20_OPENCV_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/res/sdk/example/lib/opencv4.4.0")
# 头文件定义
set(CS20_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/res/sdk/include" "${CMAKE_CURRENT_SOURCE_DIR}/res/sdk/example/thirdpart/opencv440")
# 库的寻找
file(GLOB CS20_LIB "${CS20_LIB_DIR}/*.lib")
file(GLOB CS20_OPENCV_LIB "${CS20_OPENCV_LIB_DIR}/*.lib")
set(LIB ${CS20_LIB} ${CS20_OPENCV_LIB})

include_directories(${CS20_INCLUDE_DIR})
link_directories(${CS20_LIB_DIR} ${CS20_OPENCV_LIB_DIR})
link_libraries(${LIB})

# 将源代码添加到此项目的可执行文件。
add_executable(flow_control_demo CMakeProject1)
# TODO: 如有需要，请添加测试并安装目标。
