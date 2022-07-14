cmake_minimum_required(VERSION 3.5)
project(msg_srv_ex)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

# --------------------------------------------------- #
# ------------------ ROS MSG & SRV ------------------ #
# --------------------------------------------------- #

find_package(rosidl_default_generators REQUIRED) # Find the package that generates message code from msg/srv files


# To convert the interfaces you defined into language-specific code 
# (like C++ and Python) so that they can be used in those languages
rosidl_generate_interfaces(${PROJECT_NAME}  
 "msg/Num.msg"
 "msg/AddressBook.msg"
 "srv/AddThreeInts.srv"
)

# ---- OR ---- #

# set(msg_files
#   "msg/Num.msg"
#   "msg/AddressBook.msg"
# )

# set(srv_files
#   "srv/AddThreeInts.srv"
#   )

# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}
#   ${srv_files}
# )

ament_export_dependencies(rosidl_default_runtime)

ament_package()
