cmake_minimum_required(VERSION 3.5)
project(demo_cpp_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#直接查找到对应的头文件和库文件
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include) # 包含include头文件目录

add_executable(cpp_node src/cpp_node.cpp)
add_executable(person_node src/person_node.cpp)
add_executable(learn_auto src/learn_auto.cpp)
add_executable(learn_shared_ptr src/learn_shared_ptr.cpp)
add_executable(learn_lambda src/learn_lambda.cpp)
add_executable(learn_funcational src/learn_funcational.cpp)
add_executable(learn_thread src/learn_thread.cpp)
# 第一个是生成目标文件的名字，第二个是相对目录，也就是相对cmake文件的位置 


#message(STATUS ${rclcpp_INCLUDE_DIRS}) 
#打印rclcpp头文件的路径及依赖的头文件路径
#message(STATUS ${rclcpp_LIBRARIES}) 
#打印rclcpp库文件的路径及依赖的库文件路径

# target_include_directories(cpp_node PUBLIC ${rclcpp_INCLUDE_DIRS})#头文件包含
# #第一个参数为add_executable(ros2_cpp_node ros2_cpp_node.cpp)添加的可执行文件名，第二个参数为头文件的路径 
# target_link_libraries(cpp_node ${rclcpp_LIBRARIES}) #库文件链接
# #第一个参数为add_executable(ros2_cpp_node ros2_cpp_node.cpp)添加的可执行文件名，第二个参数为库文件路径

ament_target_dependencies(cpp_node rclcpp) #与上面两句代码作用一样
ament_target_dependencies(person_node rclcpp)
target_link_libraries(learn_thread PRIVATE Threads::Threads)

install(TARGETS cpp_node person_node learn_auto learn_shared_ptr learn_lambda learn_funcational learn_thread
  DESTINATION lib/${PROJECT_NAME}
)
#表示目标是把生成的可执行文件cpp_node拷贝到目的地址：lib下的${PROJECT_NAME}也就是demo_cpp_pkg

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
