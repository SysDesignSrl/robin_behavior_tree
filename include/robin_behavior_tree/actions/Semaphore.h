#ifndef ROBIN_BEHAVIOR_TREE_SEMAPHORE_ACTION_NODE_H
#define ROBIN_BEHAVIOR_TREE_SEMAPHORE_ACTION_NODE_H
#include <string>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <behaviortree_cpp_v3/action_node.h>
// Boost
#include <boost/bind.hpp>


namespace sysdesign { namespace bt {


class Semaphore : public BT::StatefulActionNode {
private:
  const std::string LOGNAME = "Semaphore";

  ros::NodeHandle node;
  ros::Publisher request_pub;
  ros::Subscriber consensus_sub;

  bool request;
  bool consensus;

public:

  Semaphore(ros::NodeHandle &node, const std::string &name, const BT::NodeConfiguration &config)
  : node(node), BT::StatefulActionNode(name, config)
  {
    if (request_pub == nullptr)
    {
      auto topic_name = getInput<std::string>("request_topic").value();
      request_pub = node.advertise<std_msgs::Bool>(topic_name, 10, true);
    }
    if (consensus_sub == nullptr)
    {
      auto topic_name = getInput<std::string>("consensus_topic").value();
      consensus_sub = node.subscribe<std_msgs::Bool>(topic_name, 10, boost::bind(&Semaphore::consensus_cb, this, _1));
    }
  }

  void consensus_cb(const std_msgs::Bool::ConstPtr &msg)
  {
    consensus = msg->data;
  }

  static BT::PortsList providedPorts()
  {
    return
    {
      BT::InputPort<std::string>("request_topic"),
      BT::InputPort<std::string>("consensus_topic"),
    };
  }

  BT::NodeStatus onStart() override
  {
    request = true;

    std_msgs::Bool msg;
    msg.data = request;
    request_pub.publish(msg);

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (request && consensus)
    {
      request = false;

      std_msgs::Bool msg;
      msg.data = request;
      request_pub.publish(msg);

      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    request = false;

    std_msgs::Bool msg;
    msg.data = request;
    request_pub.publish(msg);
  }

};

} }  // namespace
#endif
