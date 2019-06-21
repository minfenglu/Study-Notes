# Table of Contents
- [ROS Basics](#rosbasics)
  * [Packages](#packages)
  * [Master](#master)
  * [Nodes](#nodes)
  * [Topics and Messages](#topicsmessages)
- [Writing ROS Program](#writeros)
  * [Hello World Program](#hello)
    + [Create Workspace and Package](#createwspck)
    + [Write Hello World Program](#writehelloworld)
    + [Compile Hello World Program](#compilehelloworld)
  * [Publisher Program](#pubpro)
    + [Write pubvel Program](#writepubvel)
    + [Compile pubvel Program](#compilepubevl)
    + [Execute pubvel Program](executepubevl)
  * [Subscriber Program](#subpro)
    + [Write subpose Program](#writesubpose)
- [Log Messages](#logmessage)
  * [Severity Levels](#slevel)
  * [Generate Log Messages](#genlog)
    + [Normal Log](#normallog)
    + [One-time Log](#onetimelog)
    + [Throttled Log](throttedlog)
  * [View Log Messages](#viewlog)
  * [Enable and Disable Log Messages](#enabledisablelog")
- [Graph Resource Name](#grname)

# ROS Basics <a name="rosbasics"/>

## Packages <a name="packages"></a>
* ```rospack list```
* ```rospack find package-name```
* ```rosls package-name```
* ```roscd package-name```

Each package is defined by a manifest, which is a file called package.xml. This file defines some details about the package, including its name, version, maintainer, and dependencies. The directory containing package.xml is called the package directory. (In fact, this is the definition of a ROS package: Any directory that ROS can find that contains a file named package.xml is a package directory.) This directory stores most of the pack- age’s files.

## Master <a name="master"></a>
* ```roscore```

ROS master allows nodes to communicate with one another


## Nodes <a name="nodes"></a>
* ```rosrun package-name executable-name```
* ```rosrun package-name executable-name __name:=node-name```
* ```rosnode list```
* ```rosnode info node-name``` (output a list of topics, for which taht node is a publisher or subscriber, the services offered by that node, the Linux process identifier (PID) and a summary of the connections it has made to other nodes)
* ```rosnode kill node-name```

A **running instance** of a ROS program is called a node (if we execute multiple copies of the same program at the same time, ensuring that each uses a different node name, each of the copies is treated as a separate node)

**/rosout** is a special node that is started automatically by **roscore**.

## Topcis and Messages <a name="topicsmessages"></a>

* Topic Related
  * ```rqt_graph``` (visulize the publish-subscribe relationships between ROS nodes)
  * ```rostopic list```
  * ```rostopic echo topic-name```
  * ```rostopic hz topic-name```
  * ```rostopic bw topic-name```
  * ```rostpoic info topic-name```

* Message Related
  * ```rosmsg show message-type-name``` (see details about a message type)
  * ```rostopic pub -r rate-in-hz topic-name message-type message-content``` (publish messages from command line)

Every message type belongs to a specific package. The format will be package-name/type-name  


The primary mechanism that ROS nodes use to communicate is to send messages. Messages in ROS are organized into named topics.

A node that wants to share information will publish messages on the appropriate topic or topics; a node that wants to receive information will subscribe to the topic or topics that it’s interested in.

The ROS master takes care of ensuring that publishers and subscribers can find each other; the messages themselves are sent directly from publisher to subscriber.

# Writing ROS Program <a name="writeros"></a>

## Write Hello World Program <a name="hello"></a>

### Create Workspace and Package <a name="createwspck"></a>

**Creating a workspace**: Packages should live together in a directory called a **workspace**.

We need to create a subdirectory called src inside the workspace directory to contain the source code for packages.

**Creating a package**: From inside the **src** directory run the command to create a new ROS package

catkin_create_pkg package-name

It will create a directory to hold the packages and two configurations files inside the directory
* **package.xml**: manifest file
* **CMakeLists.txt**: a script for an industrial-strength cross-platform build system called CMake. It contains a list of build instructions including what executables should be created, what source files to use to build each of them, and where to find the include files and libraries needed for those executables. CMake is used internally by catkin.

### Write Hello World program <a name="writehelloworld"></a>

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


### Compile the Program <a name = "compilehelloworld"></a>
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

### Write pubvel <a name="writepubvel"></a>
* Include the message type declaration

```#include <package_name/type_name.h>```

* Create a publisher object

```ros::Publisher pub = node_handel.advertise<message_type>(topic_name, queue_size)```

The topic_name is a string containing the name of the topic on which we want to publish. It should match the topic names shown by rostopic list or rqt_graph, but (usually) without the leading slash (/). We drop the leading slash to make the topic name a relative name

If the program rapidly publishes more messages than the queue can hold, the oldest unsent messages will be discarded.


message queue is needed because, in most cases, the message must be trans- mitted to another node. This communication process can be time consuming, especially compared to the time needed to create messages. ROS mitigates this delay by having the publish method store the message in an “outbox” queue and return right away. A separate thread behind the scenes actually transmits the message. The integer value given here is the number of messages—and not the number of bytes—that the message queue can hold.

The ROS client library is smart enough to know when the publisher and subscriber nodes are part of the same underlying process. In these cases, the message is delivered directly to the subscriber, without using any network transport. This feature is very important for making nodelets􏰃— that is, multiple nodes that can be dynamically loaded into a single process— efficient.



* create and fill the message object

* publish the message

* control the publishing rate

```ros:Rate rate(hz_rate)``` controls how rapidly the loop runs. The parameter in the constructor is in units of Hz (cycles per second). Near the end of each loop iteration, ```rate.sleep()``` is called to cause a delay in the program. The duration of the delay is calculated to prevent the loop from iterating faster than the specified rate. Without this kind of control, the program would publish messages as fast as the computer allows, which can overwhelm publish and subscribe queues and waste computation and network resources.

In extreme cases, in which the real work of the loop takes longer than the requested rate, the delay induced by ```rate.sleep()```  can be reduced to zero.

We can confirm this regulation is working correctly using ```rostopic hz```

pubvel.cpp
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

    // Check for node shutdown
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


Be mindful of the lifetime of the ros::Publisher objects. Creating the publisher is an expensive operation, so it’s a usually bad idea to create a new ros::Publisher object each time you want to publish a message. Instead, create one publisher for each topic, and use that publisher throughout the execution of your program. In pubvel, we accomplish this by declaring the publisher outside of the while loop.




Ways to make ```ros::ok()``` return false:
* use ```rosnode kill ```on the node
* send an interrupt signal (Ctrl-C) to the program
* call ```ros::shutdown()``` somewhere in the program
* start another node with the same name


### Compile pubvel Program <a name="compilepubevl"></a>
* Declare Message Type Dependencies

Because ```pubvel.cpp``` uses a message type from the ```geometry_msgs``` package, we must declare a dependency on that package. So we need to modify the find_package line in ```CMakeLists.txt``` to mention ```geometry_msgs``` in addition to ```roscpp```:

```find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)```

In ```package.xml```, we need to add elements for the new dependency:
```xml
<build_depend>geometry_msgs</build_depend>
<run_depend>geometry_msgs</run_depend>
```

### Execute pubvel Program <a name="executepubevl"></a>
```bash
rosrun demo pubvel
```

```bash
rosrun turtlesim turtlesim_node
```


## Publisher Program <a name="pubpro"></a>
### Write subpose Program <a name="writesubpose"></a>
* Write callback Function

One important difference between publishing and subscribing is that a subscriber node doesn’t know when messages will arrive. To deal with this fact, we must place any code that responds to incoming messages inside a callback function, which ROS calls once for each arriving message. A subscriber callback function looks like this:

```cpp
void function_name(const package_name::type_name &msg){
  ...
}
```
The callback function is executed each time a message arrives

* Create subscriber Object

```cpp
ros::Subscriber sub = node_handle.subscribe(topic_name, queue_size, pointer_to_callback_fuction);
```
When new messages arrive, they are stored in a queue until ROS gets a chance to execute the callback function. ```queue_size``` establishes a maximum number of messages that ROS will store in that queue at one time.

If new messages arrive when the queue is full, the oldest unprocessed messages will be dropped to make room. This may seem, on the surface, to be very similar to the technique used for publishing messages, but differs in an important way: The rate at which ROS can empty a publishing queue depends on the time taken to actually transmit the messages to subscribers, and is largely out of our control. In contrast, the speed with which ROS empties a subscribing queue depends on how quickly we process callbacks. Thus, we can reduce the likelihood of a subscriber queue overflowing by

(a) ensuring that we allow callbacks to occur, via ```ros::spin``` or ```ros::spinOnce```, frequently
(b) reducing the amount of time consumed by each callback.


The ```pointer_to_callback_fuction``` parameter is a pointer to the callback function that ROS should execute when messages arrive. In C++, we can get a pointer to a function using the ampersand (&, “address-of”) operator on the function name. (The ampersand is actually optional, and many programs omit it. The compiler can tell that we want a pointer to the function, rather than the value returned from executing the function, because the function name is not followed by parentheses.)


While creating a ```ros::Subscriber``` object, we do not explicitly mention the message type anywhere. In fact, the subscribe method is templated, and the C++ compiler infers the correct message type based on the data type of the callback function pointer we provide.

* Give ROS Control
ROS will only execute our callback function when we give it explicit permission to do so. There are two ways to do this:

```cpp
ros::spinOnce();
```
It asks ROS to execute all the pending callbacks from all the node's subscriptions, and then return control back to us.

and

```cpp
ros::spin();
```
It asks ROS to wait for and execute callbacks until the node shuts down

This is roughly equivalent to

```cpp
while(ros::ok()){
  ros::spinOnce();
}
```



```cpp
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <iomanip>

// The callback function is executed each time
// a new pose message arrives
void poseMessageReceived(const turtlesim::Pose &msg) {
    ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "position=(" << msg.x << "," << msg.y << ")" << " direction=" << msg.theta);
}

int main(int argc, char **argv){
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "subscribe_to_pose");
    ros::NodeHandle nh;

    // Create a subscriber object
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, &poseMessageReceived);

    // Let ROS take over
    ros::spin();
}
```


# Log Messages <a name="logmessage"></a>

## Severity Levels <a name="slevel"></a>
* DEBUG
* INFO
* WARN
* ERROR
* FATAL

## Generate Log Messages <a name="genlog"></a>


```c++
#include <ros/ros.h>
int main(int argc, char **argv){
    ros::init(argc, argv, "count_and_log");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    for (int i = 1; ros::ok(); i++){
        ROS_DEBUG_STREAM("Counted to : " << i);
	if (i % 3 == 0){
            ROS_INFO_STREAM(i << " is divisible by 3");
	}		
	if (i % 5 == 0){
	    ROS_WARN_STREAM(i << " is divisible by 5");
	}
	if (i % 10 == 0){
            ROS_ERROR_STREAM(i << " is divisible by 10");
	}		
	if (i % 20 == 0){
	    ROS_FATAL_STREAM(i << " is divisible by 20");
	}
	rate.sleep();
    }	    
}
```

There’s no need to use std::endl nor any other line terminator, because the logging system is already line-oriented.


To generate one-time log messages: <a name="onetimelog"></a>
```C++
ROS_DEBUG_STREAM_ONCE(message);
ROS_INFO_STREAM_ONCE(message);
ROS_WARN_STREAM_ONCE(message);
ROS_ERROR_STREAM_ONCE(message);
ROS_FATAL_STREAM_ONCE(message);
```

The first time these macros are encountered during a program’s execution, they generate the same log messages as the corresponding non-ONCE versions. After that first execution, these statements have no effect.




There are macros for throttling the rate at which a given log message appears.
To generate throttled log messages: <a name="throttedlog"></a>
```c++
ROS_DEBUG_STREAM_THROTTLE(interval, message);
ROS_INFO_STREAM_THROTTLE(interval, message);
ROS_WARN_STREAM_THROTTLE(interval, message);
ROS_ERROR_STREAM_THROTTLE(interval, message);
ROS_FATAL_STREAM_THROTTLE(interval, message);
```
The ```interval``` parameter is a double that specifies the minimum amount of time, mea- sured in seconds, that must pass between successive instances of the given log message.


## View Log Messages <a name="viewlog"></a>

There are three destinations for log messages.

* Console

```DEBUG```  and ```INFO``` are sent to standard output

```WARN```, ```ERROR``` AND ```FATAL``` messages are sent to standard error

We can format the console messages by setting ```ROSCONSOLE_FORMAT``` environment variable.

* rosout

Every log message is also published on the topic ```/rosout``` and the message type of this topic is ```rosgraph_msgs/Log```

we can use
```bash
rostopic echo /rosout
```
or

```bash
rqt_console
```

* Log Files

```/rosout``` node generates a log file. As part of its callback function for the ```/rosout``` topic, this node writes a line to a files with a name like this: ```∼/.ros/log/run_id/rosout.log```

To find the run_id, there are two ways to find out hte ```run_id``` of the current session.

### Find run_id

We can examine the output generated by ```roscore```. We can see a line like

```bash
setting /run_id to run_id
```

OR we can ask the master for the current run_id.

```bash
rosparam get /run_id
```


### Check and Purge Log Files

We can see the amount of disk space in the current user account consumed by ROS logs by using this command:

```bash
rosclean check
```

If the logs are consuming too much disk space, we can remove all the existing logs using this command:

```bash
rosclean purge
```

## Enable and Disable Log Messages <a name="enabledisablelog"></a>
### Set Logger Level

* from command line
```bash
rosservice call /node-name/set_logger_level ros.package-name level
```


 * from GUI
 ```bash
 rqt_logger_level
 ```

 * from C++ code

 ```C++
#include <log4cxx/logger.h>
log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(       
    ros::console::g_level_lookup[ros::console::levels::Debug]
);  
ros::console::notifyLoggerLevelsChanged();  
```

# Graph Resource Name <a name="grname"></a>
