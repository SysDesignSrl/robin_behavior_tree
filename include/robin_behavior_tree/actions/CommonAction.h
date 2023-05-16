#ifndef ROBIN_BEHAVIOR_TREE_COMMON_ACTION_NODE_H
#define ROBIN_BEHAVIOR_TREE_COMMON_ACTION_NODE_H
#include <string>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <robin_bridge_generated/CommonCommand.h>
#include <robin_bridge_generated/CommonFeedback.h>
// Boost
#include <boost/bind.hpp>


namespace sysdesign { namespace bt {


class CommonAction : public BT::StatefulActionNode {
private:
  const std::string LOGNAME = "CommonAction";

  ros::NodeHandle node;
  ros::Publisher command_pub;
  ros::Subscriber feedback_sub;

  BT::NodeStatus status;

  // command
  bool execute;
  bool abort;

  // feedback
  bool busy;
  bool done;
  bool error;
  bool aborted;
  int error_id;

public:

  CommonAction(ros::NodeHandle &node, const std::string &name, const BT::NodeConfiguration &config)
  : node(node), BT::StatefulActionNode(name, config)
  {
    if (command_pub == nullptr)
    {
      auto topic_name = getInput<std::string>("command_topic").value();
      command_pub = node.advertise<robin_bridge_generated::CommonCommand>(topic_name, 10, true);
    }
    if (feedback_sub == nullptr)
    {
      auto topic_name = getInput<std::string>("feedback_topic").value();
      feedback_sub = node.subscribe<robin_bridge_generated::CommonFeedback>(topic_name, 10, boost::bind(&CommonAction::feedback_cb, this, _1));
    }
  }

  void feedback_cb(const robin_bridge_generated::CommonFeedback::ConstPtr &msg)
  {
    busy = msg->busy;
    done = msg->done;
    error = msg->error;
    aborted = msg->aborted;
    error_id = msg->error_id;
  }

  static BT::PortsList providedPorts()
  {
    return
    {
      BT::InputPort<std::string>("command_topic"),
      BT::InputPort<std::string>("feedback_topic"),
      BT::InputPort<int>("command_id"),
      BT::OutputPort<int>("error_id"),
    };
  }

  BT::NodeStatus onStart() override
  {
    execute = true;
    abort = false;

    robin_bridge_generated::CommonCommand msg;
    msg.execute = execute;
    msg.abort = abort;
    msg.id = getInput<int>("command_id").value();
    command_pub.publish(msg);

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (done)
    {
      execute = false;
      abort = false;

      robin_bridge_generated::CommonCommand msg;
      msg.execute = execute;
      msg.abort = abort;
      msg.id = getInput<int>("command_id").value();
      command_pub.publish(msg);

      status = BT::NodeStatus::SUCCESS;
    }
    if (error)
    {
      execute = false;
      abort = false;

      robin_bridge_generated::CommonCommand msg;
      msg.execute = execute;
      msg.abort = abort;
      msg.id = getInput<int>("command_id").value();
      command_pub.publish(msg);

      status = BT::NodeStatus::FAILURE;
    }
    if (aborted)
    {
      execute = false;
      abort = false;

      robin_bridge_generated::CommonCommand msg;
      msg.execute = execute;
      msg.abort = abort;
      msg.id = getInput<int>("command_id").value();
      command_pub.publish(msg);

      status = BT::NodeStatus::FAILURE;
    }

    if (!execute && !busy && !done && !error && !aborted)
    {
      return status;
    }

    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    execute = false;
    abort = true;

    robin_bridge_generated::CommonCommand msg;
    msg.execute = execute;
    msg.abort = abort;
    msg.id = getInput<int>("command_id").value();
    command_pub.publish(msg);
  }

};

} }  // namespace
#endif
