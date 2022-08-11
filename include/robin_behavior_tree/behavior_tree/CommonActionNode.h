#ifndef ROBIN_BEHAVIOR_TREE_COMMON_ACTION_NODE_H
#define ROBIN_BEHAVIOR_TREE_COMMON_ACTION_NODE_H
// STL
#include <string>
#include <vector>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <robin_bridge_generated/CommonCommand.h>
#include <robin_bridge_generated/CommonFeedback.h>
// BehaviorTree.CPP
#include <behaviortree_cpp_v3/action_node.h>
// Boost
#include <boost/bind.hpp>


namespace sysdesign { namespace bt {


class CommonActionNode : public BT::StatefulActionNode {
private:
  const std::string LOGNAME = "CommonActionNode";

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

  CommonActionNode(ros::NodeHandle &node, const std::string &name, const BT::NodeConfiguration &config)
  : node(node), BT::StatefulActionNode(name, config)
  {
    command_pub = node.advertise<robin_bridge_generated::CommonCommand>("command", 10, true);
    feedback_sub = node.subscribe<robin_bridge_generated::CommonFeedback>("feedback", 10, boost::bind(&CommonActionNode::feedback_cb, this, _1));
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
