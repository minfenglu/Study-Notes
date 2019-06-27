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
  * [Global Names](#globalname)
  * [Relative Names](#relativename)
  * [Private Names](#privatename)
  * [Anonymous Names](#anonymousname)
- [Launch Files](#launchfile)
  * [Use Launch Files](#uselaunch)
  * [Create Launch Files](#createlaunch)
    + [Where to Place Launch Files](#launchplace)
    + [Basic Ingredients](#basicingredients)
    + [Remappings](#remappings)
    + [Include Other Files](#includetoherlaunch)
    + [Launch Arguments](#launcharguments)
    + [Create Groups](#creategroups)
- [Parameters](#paramters)

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

Nodes, topics, services, and parameters are collectively referred to as ```graph resources```.Every graph resource is identified by a short string called a graph resource name.

## Global Names <a name="globalname"></a>
Global names have clear, unambiguous meanings and need no additional context information.

There are several parts to a global name:
* A leading ```/``` to identify the name as global name.

* A sequence of zero or more namespaces, separated by slashed.

* A base name that describes the resource itself.  

## Relative names <a name="relativename"></a>
A relative name allows ROS to supply a default namespace. It cannot be matched to specific graph resources unless we know the default namespace the ROS is using to resolve the relative name

The default namespace is tracked individually for each node rather than a system-wide setting.  If the default namespace is not set, ROS will use the global namespace ```/```.

The best and most common way to choose a default namespace for a node is to use ```ns``` attributes in a launch file. However, there are some other ways to do it manually

* Most ROS programs that call ```ros::init``` accept a command line parameter ```__ns```, which specifies a default namespace for that program

* We can also set the default namespace for every ROS program executed within a shell, using an environment variable

```bash
export ROS_NAMESPACE=default-namespace
```


The real value of relative names is that they make it easier to build complicated systems by composing smaller parts.

When a node uses relative names, it is essentially giving its users the ability to easily push that node and the topics it uses down into a namespace that the node’s original designers did not necessarily anticipate.

This kind of flexibility can make the organization of a system more clear and, more importantly, can prevent name collisions when groups of nodes from different sources are combined.

## Private Names <a name="privatename"></a>

Each node has its own namespace for things that are related only to that node, and are not interesting to anyone else. Private names are often used for parameters. ```roslaunch``` has a specific feature for setting parameters that are accessible by private names.

Private names begin with a tilde (~) and don't fully specify the namespace in which they live.

Instead of using the current default namespace, private names use the name of their node as a namespace.


For instance, in a node whose global name is ```/sim1/pubvel```, the private name ```~max_vel``` would be converted to global name ```/sim1/pubvel/max_vel```


## Anonymous Names <a name="anonymousname"></a>
Anonymous names make it easier to obey the rule that each node must have a unique name. During its coll to ```ros::init```, a node can request that a unique name be assigned automatically.

To request an anonymous name, a node should pass ```ros:::init_options::AnonymoustName``` as a fourth paramter to ```ros::init```. The effect of this extra option is to append some extra text to the given base name, ensuring that the node’s name is unique. ```ros::init``` uses the current wall clock time to form anonymous names.



```c++
ros;:init(argc, argc, base_name, ros::init_options::AnonymoustName)
```



```c++
// This program allows multiple copies to execute at the same time
// without needing to manually create distinct names
// for each of them
#include <ros/ros.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "anon", ros::init_options::AnonymoustName);
  ros::NodeHandle nh;
  ros::Rate rate(1);
  while(ros::ok()){
    ROS_INFO_STREAM("This message is from " << ros::this_node::getName());
    rate.sleep();
  }
}
```

# Launch Files <a name="launchfile"></a>

ROS provides a mechanism for starting the master and many nodes all at once using a launch file. The basic idea is to list a group of nodes that should be started at the same time in a specific XML format.

## Use Launch Files <a name="uselaunch"></a>
* execute launch files
```bash
roslaunch package-name launch-file-name
```

example.launch

```XML
<launch>
  <node
     pkg="turtlesim"
     type="turtlesim_node"
     name="turtlesim "
     respawn="true"
  />
  <node
     pkg="turtlesim"
     type="turtle_teleop_key"
     name="teleop_key "
     required="true"
     launch−prefix="xterm −e"
  />
  <node
     pkg="demo"
     type="subpose"
     name="pose_subscriber"
     output="screen"
  />
</launch>
```

We can invoke the example launch file using this command:
```bash
roslaunch demo example.launch
```

It is also possible to use launch files that are not part of any package. To do this, give roslaunch only the path to the launch file, without mentioning any package.

```bash
roslaunch ∼/ros/src/demo/example.launch
```


An important fact about ```roslaunch``` is that all of the nodes in a launch file are started at roughly the same time. As a result, we cannot be sure about the order in which the nodes will initialize themselves. Well-written ROS nodes don’t care about the order in which they and their siblings start up.


## Create Launch file <a name="createlaunch"></name>
### Where to Place Launch Files <a name="launchplace"></a>
Each launch file should be associated with a particular package. The usual naming scheme is to give launch files names ending with ```.launch```.

The simplest place to store launch files is in the package directory. When looking for launch files, ```roslaunch``` will also search subdirecotries of each package directory. Some packages utilize this feature by organizing launch files into a subdirecotry of their own, usually called ```launch```

### Basic Ingredients <a name="basicingredients"></a>
#### Root Element

For ROS launch files, the root element is defined by

```XML
<launch>
  ...
</launch>
```

#### Nodes
Each of the node elements names a single node to launch

```XML
<node
  pkg="package-name"
  type="executable-name"
  name="node=name"
/>   
```  

The  ```name``` attribute assigns a name to the node and overrides any name that the node would normally assign to itself in its call to ```ros::init```

To use an anonymous name from within a launch file, use an ```anon``` substitution. Note: multiple uses of the same base name will generate the same anonymous name.

```
name="$(anon base_name)"
```


#### Find Node Log File
By default, standard output from launched nodes is redirected to a log file and does not appear on the console. The name of the log file is
```
~/.ros/log/run_id/node_name-number-stdout.log
```

```
turtlesim-1-stdout.log
telep_key-3-stdout.log
```
To override this behavior for a single node, use the ```output``` attribute in its node element

```
output="screen"
```

We can also force roslaunch to display output from all of its nodes using the ```--screen``` command-line option

```
roslaunch --screen package-name launch-file-name
```

#### Request Respawning
After starting all the requested nodes, ```roslaunch``` monitors each node, keeping track of which ones remain active. For each node, we can asl ```roslaunch``` to restart it when it terminates by using a ```respawn``` attribute:

```
respawn="true"
```

#### Require Nodes
An alternative to ```respawn``` is to declare that a node is required:  

```
required="true"
```

When a required node terminates, ```roslaunch``` responds by terminating all of the other active nodes and exiting itself.

#### Launch Nodes in Their Own Windows
One potential drawback to using ```roslaunch``` is that all of the nodes share the same terminal. For nodes that do rely on console input, it may be preferable to retain the separate terminals.

```
launch-prefix="command-prefix"
```

```roslaunch``` will insert the given prefix at the start of the command line it constructs internally to execute the given node

In the ```example.launch```, we use ```launch-prefix="xterm -e"```. this node element is roughly equivalnent to this command:
```
xterm -e rosrun turtlesim turtle_teleop_key
```

The ```xterm``` command starts a simple terminal window. The ```-e``` argument tells xterm to execute the remainder of its command line


#### Launch Nodes Inside A Namespace

The usual way to set the default namespace for a node—a process often called pushing down into a namespace—is to use a launch file, and assign the ```ns``` attribute in its node element

```
ns="namespace"
```

```roslaunc``` requires the node names in the launch files to be base names-relative names without any namespaces

```xml
<launch>
  <node
    name="turtlesim_node"
    pkg="turtlesim"
    type="turtlesim_node"
    ns="sim1"
  />
  <node
    pkg="turtlesim"
    type="turtle_teleop_key"
    name="teleop_key "
    required="true"
    launch−prefix="xterm −e"
    ns="sim1"
  />
  <node
    name="turtlesim_node"
    pkg="turtlesim"
    type="turtlesim_node"
    ns="sim2"
  />
  <node
    pkg="demo"
    type="pubvel"
    name="velocity_publisher"
    ns="sim2"
  />
</launch>
```
doublesim.launch <a name="doublesimlaunch"></>

```doublesim.launch``` starts two independent turtlesim simulations. One simulation has a turtle moved by randomly-generated velocity commands; the other is teleoperated.


### Remap Names <a name="remapname"></a>

#### Create Remappings
* remap a name when starting a node from the command line

```
original-name:=new-name
```

```bash
rosrun turtlesim turtlesim_node tutle1/pose:=tim
```
* remap  names within a launch file

```XML
<remap from="original-name" to="new-name" />
```

If it appears at the top level, as a child of the launch element, this remapping will apply to all subsequent nodes. These remap elements can also appear as children of a node element, like this:
```xml
<node node-attributes >
  <remap from="original-name" to="new-name" />
  ...
</node>
```



Consider a scenario in which we want to use turtle_teleop_key to drive a turtlesim turtle, but with the meanings of arrow keys reversed.



```c++
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

ros::Publisher *pubPtr;
void commandVelocityReceived(
  const geometry_msgs::Twist& msgIn
){
  geometry_msgs::Twist msgOut;
  msgOut.linear.x = -msgIn.linear.x;
  msgOut.angualr.z = -msgIn.angualr.z;
  pubPtr->publish(msgOut);
}


int main(int argc, char **argv){
  ros::init(argc, argc, "reverse_velocity");
  ros::NodeHandle nh;

  pubPtr = new ros::Publisher(
    node.advertise<geometry_msgs::Twist>(
      "turtle1/cmd_vel_reversed",
      1000));

  ros::Subscriber sub = nh.subscribe(
    "turtle1/cmd_vel", 1000,
    &commandVelocityReceived);
  ros::spin();

  delete pubPtr;
}

```
reverse_cmd_vel.cpp



```XML
<launch>
  <node
    pkg="turtlesim"
    type="turtlesim_node"
    name="turtlesim"
  >
    <remap
      from="turtle1/cmd_vel"
      to="turtle1/cmd_vel_reversed"
    />
  </node>  
  <node
    pkg="turtlesim"
    type="turtle_teleop_key"
    name="teleop_key "
    launch−prefix="xterm −e"
  />
  <node
    pkg="demo"
    type="reverse_cmd_vel"
    name="reverse_velocity"
  />
</launch>
```
reverse.launch


### Include Other Files <a name="includetoherlaunch"> </a>
To include contents of another launch file, including all of its nodes and parameters:
```
<include file="path-to-launch-file"/>
```

The file attribute expects the full path to the file we want to include. Most ```include``` elements use a find substitution to search for a package, instead of explicitly naming a direcotry:

```
<include file="$(find package-name)/launch-file-name"/>
```

The ```find``` argument is expanded via a string substitution to the path to the given package.

The ```include``` element also supports the ```ns``` attribute for pushing its contents into a namespace
```
<include file=". . . " ns="namespace" />
```

### Launch Arguments
To make launch files configurable, ```roslaunch``` supports launch arguments.

The example launch file uses ```use_sim3``` as argument to determine whether to start three copies of turtlesim or two. See [doublesim.launch](#doublesimlaunch)


```xml
<launch>
  <include
    file="$(find demo)/doublesim.launch"
  />
  <arg
    name="use_sim3"
    default="0"
  />

  <group ns="sim3" if="$(arg use_sim3)" >
    <node
      name="turtlesim_node"
      pkg="turtlesim"
      type="turtlesim_node"
    />
    <node
      pkg="turtlesim"
      type="turtle_teleop_key"
      name="teleop_key "
      required="true"
      launch−prefix="xterm −e"
    />
  </group>
</launch>
```
triplesim.launch


Note: arguments make sense only within launch files; their values are not directly available to nodes.

#### Declare Arguments
```
<arg name="arg-name" />
```

#### Assign Argument Values
```
roslaunch package-name launch0file-name arg-name:=arg-vale
```

Alternatively, we can provide a vlue as part of the arg declaration.
```
<arg name="arg-name" default="arg-value" />
<arg name="arg-name" value="arg-value" />
```


Note: A command line argument can override a default, but not a value.


#### Access Argument Values
```
$(arg arg-name)
```

```roslaunch``` will replace it with the value of the given argument

#### Send Argument Values to Included Launch Files

```
<include file="path-to-launch-file">
  <arg name="arg-name" value="arg-value"/>
  ...
</include>
```


One common scenario is that both launch files—the included one and the including one—have some arguments in common. In such cases, we might want to pass those values along unchanged. An element like this, using the same argument name in both places, does this:

```
<arg name="arg-name" value="$(arg arg-name)" />
```

The first appearance of the argument’s name refers to that argument in the included launch file. The second appearance of the name refers to the argument in the including launch file.

#### Create Groups <a name="creategroups"></a>

```group``` provides a convenient way to organize nodes in a large launch file. The ```group``` element can serve two purposes:
* push several nodes into the same namespace

```
<group ns="namespace">
  ...
</group>
```

* conditionally enable or disable nodes

```
<group if="0-or-1">
  ...
</group>
```
If the value of the if attribute is 1, then the enclosed elements are included normally. If this attribute has value 0, then the enclosed elements are ignored. The unless attribute works similarly, but with the meanings reversed:

```
<group unless="1-ot-0">
  ...
</group>
```

# Parameters <a name="parameters"

A centralized parameter server keeps track of a collection of values, like integers, floating point numbers, strings, or other data, each of which is identified by a short string name.

Because parameters must be actively queried by the nodes that are interested in their values, they are most suitable for configuration information that will not change (much) over time.

## Access Parameters from Command Line  

```bash
rosparam list
```

The output will show a list of strings and each of these strings is a name (a global graph resource name) that the parameter server has associated with some values.

## Query Parameters
```bash
rosparam get paramter_name
```
It is also possible to retrive values of every parameter in a namespace

```bash
rosparam get namespace
```

## Set Parameter
```bash
rosparam set parameter_name parameter_value
```

## Create and Load Paramter Files
To store all of the parameters from a namespace, in YAML format, to a file, use ```rosparam dump```:

```bash
rosparam dump filename namespace
```

To read parameters from a file and adds them to the
parameter server:

```bash
rosparam load filename namespace
```

Note: For both of these commands, the namespace argument is optional, and defaults to the global namespace (/).


## Example
If we start ```roscore``` and ```turtlesim_node``` and run ```rosparam list```, we will get an output like this:

```
/background_b
/background_g
/background_r
/rosdistro /roslaunch/uris/host_donatello__59636 /rosversion
/run_id
```

To set the background color to bright yellow

```
rosparam set /background_r 255
rosparam set /background_g 255
rosparam set /background_b 0
rosservice call /clear
```

Updated parameter values are not automatically “pushed” to nodes. Instead, nodes that care about changes to some or all of their parameters must explicitly ask the parameter server for those values.

 if we expect to change the values of parameters used by an active node, we must be aware of how (or if) that node requeries its parameters.  

## Access Paramters from C++
The C++ interface to ROS parameters is:

```c++
void ros::param::set(parameter_name, input_value);
bool ros::param::get(parameter_name, output_value);
```

```c++
// This program waits for a turtlesim to start up, and
// changes its background color.
#include <ros/ros.h>
#include <std_srvs/Empty.h>
int main(int argc , char** argv) {
  ros::init(argc, argv, "set_bg_color");
  ros::NodeHandle nh;

  ros::service::waitForService("clear");

  ros::param::set("background_r", 255);
  ros::param::set("background_g", 255);
  ros::param::set("background_b", 0);

  ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("/clear");
  std_srvs::Empty srv;
  clearClient.call(srv);
```
set_bg_color.cpp



```c++
// This program publishes random velocity commands, using
// a maximum linear velocity read from a parameter.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib .h>
int main(int argc , char** argv) {
  ros::init(argc, argc, "publish_velocity");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
    "turtle1/cmd_vel", 1000);
  srand(time(0));

  // Get the maximum velocity parameter.
  const std::string PARAM_NAME = "~max_vel";
  double maxVel;
  bool ok = ros::param::get(PARAM_NAME, maxVel);

  if (!ok){
    ROS_FATAL_STREAM("Could not get parameter " << PARAM_NAME); exit(1);
  }

  ros::Rate rate(2);
  while(ros::ok()){
    // Create and send a random velocity command.
    geometry_msgs::Twist msg ;
    msg.linear.x = maxVel * double(rand()) / double(RAND_MAX); msg.angular.z = 2 * double(rand()) / double(RAND_MAX) − 1;
    pub.publish(msg);

    rate.sleep();
  }

## Set Parameters in Launch Files

To ask ```roslaunch``` to set a parameter value, use a ```param``` element

```
<param name="param-name" value="param-value" />
```

```
<group ns="duck_colors">
  <param name="huey" value="red" />
  <param name="dewey" value="blue" />
  <param name="louie" value="green" />
  <param name="webby" value="pink" />
</group>
```

To set private paramters:

```
<node ...>
<param name="param-name" value="param-value" />
  ...
</node>
```


```
<node
  pkg="agitr"
  type="pubvel_with_max"
  name="publish_velocity" />
  <param name="max_vel" value="3" />
</node>
```

## Read Parameters from A File
```
<rosparam command="load" file="path-to-param-file" />
```
The parameter file listed here is usually one created by ```rosparam``` dump. It is typical to use a find substitution to specify the file name relative to a package directory

```
<rosparam
  command="load"
  file="$(find package-name)/param-file"
/>
```

```xml
<launch>
  <node
    pkg="turtlesim"
    type="turtlesim_node"
    name="turtlesim"
  />
  <node
    pkg="agitr"
    type="pubvel_with_max"
    name="publish_velocity"
  >
    <param name="max_vel" value="3" />
  </node>
  <node
    pkg="agitr"
    type="set_bg_color"
    name="set_bg_color"
  />
</launch>
```
