#ifndef ROBIN_BEHAVIOR_TREE_PROCESS_CODE_ACTION_NODE_H
#define ROBIN_BEHAVIOR_TREE_PROCESS_CODE_ACTION_NODE_H
#include <string>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int32.h>
#include <behaviortree_cpp_v3/action_node.h>
// Boost
#include <boost/bind.hpp>


namespace sysdesign { namespace bt {


class ProcessCode : public BT::StatefulActionNode {
private:
  const std::string LOGNAME = "ProcessCode";

  ros::NodeHandle node;
  ros::Publisher process_code_pub;
  ros::Subscriber process_code_sub;

  int process_code;
  int process_code_ack;

public:

  ProcessCode(ros::NodeHandle &node, const std::string &name, const BT::NodeConfiguration &config)
  : node(node), BT::StatefulActionNode(name, config)
  {
    if (process_code_pub == nullptr)
    {
      auto topic_name = getInput<std::string>("process_code_topic").value();
      process_code_pub = node.advertise<std_msgs::Int32>(topic_name, 10, true);
    }
    if (process_code_sub == nullptr)
    {
      auto topic_name = getInput<std::string>("process_code_ack_topic").value();
      process_code_sub = node.subscribe<std_msgs::Int32>(topic_name, 10, boost::bind(&ProcessCode::process_code_cb, this, _1));
    }
  }

  void process_code_cb(const std_msgs::Int32::ConstPtr &msg)
  {
    process_code_ack = msg->data;
  }

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
    process_code = getInput<int>("process_code").value();;

    std_msgs::Int32 msg;
    msg.data = process_code;
    process_code_pub.publish(msg);

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (process_code == process_code_ack)
    {
      std_msgs::Int32 msg;
      msg.data = process_code;
      process_code_pub.publish(msg);

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
