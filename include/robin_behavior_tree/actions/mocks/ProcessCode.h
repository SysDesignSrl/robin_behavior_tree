#ifndef ROBIN_BEHAVIOR_TREE_PROCESS_CODE_MOCK_ACTION_NODE_H
#define ROBIN_BEHAVIOR_TREE_PROCESS_CODE_MOCK_ACTION_NODE_H
#include <string>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int32.h>
#include <behaviortree_cpp_v3/action_node.h>


namespace sysdesign { namespace bt {


class ProcessCode : public BT::StatefulActionNode {
private:
  const std::string LOGNAME = "ProcessCode";

  ros::NodeHandle node;

  int process_code;
  int process_code_ack;

public:

  ProcessCode(ros::NodeHandle &node, const std::string &name, const BT::NodeConfiguration &config)
  : node(node), BT::StatefulActionNode(name, config) { }


  static BT::PortsList providedPorts()
  {
    return
    {
      BT::InputPort<std::string>("process_code_topic"),
      BT::InputPort<std::string>("process_code_ack_topic"),
      BT::InputPort<int>("process_code")
    };
  }

  BT::NodeStatus onStart() override
  {
    process_code = getInput<int>("process_code").value();
    process_code_ack = process_code;

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    ROS_DEBUG_NAMED(LOGNAME, "process_code: %d", process_code);
    ROS_DEBUG_NAMED(LOGNAME, "process_code_ack: %d", process_code_ack);

    if (process_code == process_code_ack)
    {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    // std_msgs::Int32 msg;
    // msg.data = process_code;
    // process_code_pub.publish(msg);
  }

};

} }  // namespace
#endif
