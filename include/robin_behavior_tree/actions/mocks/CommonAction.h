#ifndef ROBIN_BEHAVIOR_TREE_COMMON_ACTION_MOCK_NODE_H
#define ROBIN_BEHAVIOR_TREE_COMMON_ACTION_MOCK_NODE_H
#include <string>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <robin_bridge_generated/CommonCommand.h>
#include <robin_bridge_generated/CommonFeedback.h>


namespace sysdesign { namespace bt {


class CommonAction : public BT::StatefulActionNode {
private:
  const std::string LOGNAME = "CommonAction";

  ros::NodeHandle node;

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
  : node(node), BT::StatefulActionNode(name, config) { }

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

    busy = false;
    done = true;
    aborted = false;
    error = false;
    error_id = 0;

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    ROS_DEBUG_NAMED(LOGNAME, "busy: %s", (busy) ? "True" : "False");
    ROS_DEBUG_NAMED(LOGNAME, "done: %s", (done) ? "True" : "False");
    ROS_DEBUG_NAMED(LOGNAME, "error: %s", (error) ? "True" : "False");
    ROS_DEBUG_NAMED(LOGNAME, "aborted: %s", (aborted) ? "True" : "False");

    if (done)
    {
      execute = false;
      abort = false;
      done = false;
      status = BT::NodeStatus::SUCCESS;
    }
    if (error)
    {
      execute = false;
      abort = false;
      done = false;
      status = BT::NodeStatus::FAILURE;
    }
    if (aborted)
    {
      execute = false;
      abort = false;
      done = false;
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
  }

};

} }  // namespace
#endif
