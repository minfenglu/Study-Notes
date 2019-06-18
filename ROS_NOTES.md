# Table of contents
1. [ROS Basics](#rosbasics)
  1. [Packages](#packages)
  2. [Master](#master)
  3. [Nodes](#nodes)
  4. [Topics and Messages](#topicsmessages)
2. [Writing ROS Program](#writeros)
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
