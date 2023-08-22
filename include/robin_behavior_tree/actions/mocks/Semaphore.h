#ifndef ROBIN_BEHAVIOR_TREE_SEMAPHORE_MOCK_ACTION_NODE_H
#define ROBIN_BEHAVIOR_TREE_SEMAPHORE_MOCK_ACTION_NODE_H
#include <string>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <behaviortree_cpp_v3/action_node.h>


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
  : node(node), BT::StatefulActionNode(name, config) { }

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
    consensus = true;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (request && consensus)
    {
      request = false;
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    request = false;
  }

};

} }  // namespace
#endif
