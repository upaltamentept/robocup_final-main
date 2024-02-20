# How to use this msg

Locate into your package which uses this message,

## In CMakeList.txt

Find this part:

```txt
find_package(catkin REQUIRED COMPONENTS
)
```

add:

```txt
message_generation
project_msgs
```

Find

```txt
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
    moveit_core
    moveit_ros_planning_interface
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)
```

in CATKIN_DEPENDS, add

```txt
message_runtime
geometry_msgs
```

## In package.xml

Add:

```txt
<build_depend>geometry_msgs</build_depend>
<build_depend>message_generation</build_depend>

<run_depend>geometry_msgs</run_depend>
<run_depend>roscpp</run_depend>
<run_depend>rospy</run_depend>
<run_depend>message_runtime</run_depend>
<run_depend>project_msgs</run_depend>
```
