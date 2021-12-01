# ROS1 vs ROS2

## Introduction

The aim of this document is not to give in-depth details as to all the changes between ROS1 and ROS2. However, this document will give a high-level overview of some of the more important changes. 

Please note, a large amount of the following has been taken from [Roboticsbackend](https://roboticsbackend.com/ros1-vs-ros2-practical-overview/) and has been re-written for the author to gain a better understanding of the changes. 

## Why was ROS2 created?

ROS1 was created in 2007 and has quickly grown due to the open source community. However, the team behind ROS has learnt of several features that are missing. Unfortunately adding these missing features would most likely result in unstable builds. As a result, ROS2 has been built from scratch with these missing features in mind.


## C++ vs Python

To create nodes in C++ you need to import `roscpp` and for Python you need `rospy`. These two libraries are independent of each other being that some features in one package can be missing from the other.

In ROS2, this independence has been completely removed and replaced by `rcl` which has been implemented in pure C. However you won't import `rcl` directly but use libraries that have been built off of the `rcl` library. For example, if you're using C++ you will import `rclcpp` and for Python `rclpy`.

Advantages:
- Nodes are now much more similar between code bases
- Easier to create new libraries for different languages 

Versions:
- ROS1 was mainly aimed at Python2 with the exception of ROS1 Noetic which does support Python3
- ROS1 was mainly aimed at C++14, however it is possible to get C++17 if you're good at handling dependencies 

# Nodes

When creating nodes in ROS1, there is no strict format to follow. This results in lots of uniquely created nodes. However, everything has changed in ROS2. Nodes created in ROS2 have to follow strict conventions dictated by the Node object in which all node classes have to inherit from.

Advantages:
- Much cleaner code
- Easier to follow and understand
- Cooperation between developers
  
Examples:
1. Creating nodes in C++ for ROS1 vs ROS2

ROS1
```
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);

  int count = 0;
  std_msgs::String msg;
  while (ros::ok()) {
    std::stringstream ss;
    ss << "hello world " << count++;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
```

ROS2
```
#include <sstream>
#include "rclcpp/rclcpp.hpp" // CHANGED
#include "std_msgs/msg/string.hpp" // CHANGED
int main(int argc, char **argv) {
  rclcpp::init(argc, argv); // CHANGED
  auto node = rclcpp::Node::make_shared("talker"); // CHANGED
  auto chatter_pub = node->create_publisher<std_msgs::msg::String>("chatter", 1000); // CHANGED
  rclcpp::Rate loop_rate(10); // CHANGED

  int count = 0;
  std_msgs::msg::String msg; // CHANGED
  while (rclcpp::ok()) { // CHANGED
    std::stringstream ss;
    ss << "hello world " << count++;
    msg.data = ss.str();
    RCLCPP_INFO(node->get_logger(), "%s\n", msg.data.c_str()); // CHANGED
    chatter_pub.publish(msg);
    rclcpp::spin_some(node); // CHANGED
    loop_rate.sleep();
  }
  return 0;
}
```

Key changes explained:

`auto node = rclcpp::Node::make_shared("talker");`
"Instead of passing the node’s name to the library initialization call, we do the initialization, then pass the node name to the creation of the node object" - [Ros.org](https://docs.ros.org/en/foxy/Contributing/Migration-Guide.html)



2. Subscribers 

I've skipped over Publishers as they are virtually identical to ROS1 publishers with a few minor changes (As the node is now a smart pointer). However, `Subscribers` are a bit more complicated. 

As far as I can tell (more research is needed), creating a standalone call-back is no longer possible. From what I can tell, all subscribers need to be part of a standalone class which inherits from the Node object. 

ROS1 - (taken from [Ros.org](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29))
```
#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();

  return 0;
}
```

ROS2 - (taken from [hatenablog](https://tshell.hatenablog.com/entry/2020/02/16/194136)) 
```
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class Subscriber : public rclcpp::Node {
public:
    Subscriber() : Node("subscriber_node") {
        subscriber_ = this->create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&Subscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Subscribing: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Subscriber>());
    rclcpp::shutdown();
    return 0;
}
```

I have omitted the the above examples in Python as I personally believe if you can understand the above C++ code then understanding the changes in Python is a lot easier. But for your reference, here are some links to creating `Nodes` and `Subscribes` in Python: [Creating nodes in Python](https://roboticsbackend.com/write-minimal-ros2-python-node/), [Creating subscriber in Python](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

## Sources

1. [Roboticsbackend](https://roboticsbackend.com/ros1-vs-ros2-practical-overview/)
2. [Ros.org](https://docs.ros.org/en/foxy/Contributing/Migration-Guide.html)
3. [hatenablog](https://tshell.hatenablog.com/entry/2020/02/16/194136)