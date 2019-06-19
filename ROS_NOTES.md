# Table of contents
- [ROS Basics](#rosbasics)
  * [Packages](#packages)
  * [Master](#master)
  * [Nodes](#nodes)
  * [Topics and Messages](#topicsmessages)
- [Writing ROS Program](#writeros)
  * [Create Workspace and Package](#createwspck)
  * [Hello World Program](#helloworld)
  * [Compile the Program](#compile)
  * [Publisher Program](#pubpro)
# ROS Basics <a name="rosbasics"/>

## Packages <a name="packages"></a>
* rospack list
* rospack find package-name
* rosls package-name
* roscd package-name

Each package is defined by a manifest, which is a file called package.xml. This file defines some details about the package, including its name, version, maintainer, and dependencies. The directory containing package.xml is called the package directory. (In fact, this is the definition of a ROS package: Any directory that ROS can find that contains a file named package.xml is a package directory.) This directory stores most of the pack- age’s files.

## Master <a name="master"></a>
* roscore

ROS master allows nodes to communicate with one another


## Nodes <a name="nodes"></a>
* rosrun package-name executable-name
* rosrun package-name executable-name \_\_name:=node-name
* rosnode list
* rosnode info node-name (output a list of topics, for which taht node is a publisher or subscriber, the services offered by that node, the Linux process identifier (PID) and a summary of the connections it has made to other nodes)
* rosnode kill node-name

A **running instance** of a ROS program is called a node (if we execute multiple copies of the same program at the same time, ensuring that each uses a different node name, each of the copies is treated as a separate node)

**/rosout** is a special node that is started automatically by **roscore**.

## Topcis and Messages <a name="topicsmessages"></a>

* Topic Related
  * rqt_graph (visulize the publish-subscribe relationships between ROS nodes)
  * rostopic list
  * rostopic echo topic-name
  * rostopic hz topic-name
  * rostopic bw topic-name
  * rostpoic info topic-name

* Message Related
  * rosmsg show message-type-name (see details about a message type)
  * rostopic pub -r rate-in-hz topic-name message-type message-content (publish messages from command line)

Every message type belongs to a specific package. The format will be package-name/type-name  


The primary mechanism that ROS nodes use to communicate is to send messages. Messages in ROS are organized into named topics.

A node that wants to share information will publish messages on the appropriate topic or topics; a node that wants to receive information will subscribe to the topic or topics that it’s interested in.

The ROS master takes care of ensuring that publishers and subscribers can find each other; the messages themselves are sent directly from publisher to subscriber.

# Writing ROS Program <a name="writeros"></a>

## Create Workspace and Package <a name="createwspck"></a>

**Creating a workspace**: Packages should live together in a directory called a **workspace**.

We need to create a subdirectory called src inside the workspace directory to contain the source code for packages.

**Creating a package**: From inside the **src** directory run the command to create a new ROS package

catkin_create_pkg package-name

It will create a directory to hold the packages and two configurations files inside the directory
* **package.xml**: manifest file
* **CMakeLists.txt**: a script for an industrial-strength cross-platform build system called CMake. It contains a list of build instructions including what executables should be created, what source files to use to build each of them, and where to find the include files and libraries needed for those executables. CMake is used internally by catkin.

## Hello World program <a name="helloworld"></a>

package.xml

```xml
<?xml version="1.0"?>
<package>
    <name>demo</name>
    <version >0.0.1</version>
    <description>
        ROS Demo
    </description>
    <maintainer email="contact.minfeng@gmail.com">
        Minfeng Lu
    </maintainer>
    <license>TODO</license>
    <buildtool_depend>catkin</buildtool_depend> <build_depend>geometry_msgs</build_depend> <run_depend>geometry_msgs</run_depend>
    <build_depend>turtlesim</build_depend>
    <run_depend>turtlesim</run_depend>
</package>
```


hello.cpp

```c++
#include <ros/ros.h>

int main(int argc, char **argv){
    // initializes the ROS client library
    // Call this once at the beginning of the program
    // The last parameter is a string containing the default name of the node.
    ros::init(argc, argv, "hello_ros");

    // Creating this object registers the program as a node with the ROS master
    ros::NodeHandle nh;

    // Send output as log message
    ROS_INFO_STREAM("Hello, ROS!");
}
```


## Compile the Program <a name = "compile"></a>
* Declare dependencies

```txt
# default version
find_package(catkin REQUIRED)

# dependencies on other catkin packages can be added in a COMPONENTS section
find_package(catkin REQUIRED COMPONENTS package-names)
```

we should also list dependencies in the package manifest (package.xml), using **build_depend** and **run_depend**
```txt
<build_depend>package-name</build_depend>
<run_depend>package-name<run_depend>
```
* Declare an executable

```txt
# declares the name of the executable and
# a list of source files that should be combined to form that executable
# list all source files separated by spaces
add_executable(executable-name source-files)

# tells CMake to use the appropriate library flags
# defined by the find_package
target_link_libraries(executable-names ${catkin_LIBRARIES})
```

CMakeLists.txt

```txt
# What version of CMake is needed?
cmake_minimum_required (VERSION 2.8.3)

# Name of this package
project(demo)
# Find the catkin build system, and any other packages on
# which we depend.
find_package (catkin REQUIRED COMPONENTS roscpp)

# Declare our catkin package
catkin_package ()

# Specify locations of header files
include_directories
(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files . If
# there are multiple executables , use multiple copies of
# thisline.
add_executable(hello hello.cpp)

# Specify libraries against which to link . Again , this
# line should be copied for each distinct executable in
# the package.
target_link_libraries ( hello ${catkin_LIBRARIES})
```



* Build workspace

```bash
cakin_make
```
This command must be run from the workspace directory. It will create **devel** and **build** subdirectories within the workspace.



* Source

```bash
source devel/setup.bash
```

* Execute

```bash
rosrun demo hello
```

## Publisher Program <a name="pubpro"></a>

* Include the message type declaration
* Create a publisher object
```c++
// every ROS topic is associated with a message type.
// Each message type has a corresponding C++ header file.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "publish_velocity");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

    // Seed the random number generator
    srand(time(0));

    // Loop at 2Hz until the node is shut down
    ros::Rate rate(2);

    while(ros::ok()) {
        geometry_msgs::Twist msg;
	      msg.linear.x = double(rand())/double(RAND_MAX);
	      msg.angular.z = 2*double(rand())/double(RAND_MAX) - 1;

	      pub.publish(msg);

	      ROS_INFO_STREAM("Sending random velocity command: "
          << "linear=" << msg.linear.x
          << " angualr=" << msg.angular.z);

        // Wait until it's time for another iteration     
	      rate.sleep();
    }
}
```
