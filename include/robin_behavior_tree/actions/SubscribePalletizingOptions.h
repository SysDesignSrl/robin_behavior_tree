#ifndef ROBIN_BEHAVIOR_TREE_SUBSCRIBE_PALLETIZING_OPTIONS_TOPIC_NODE_H
#define ROBIN_BEHAVIOR_TREE_SUBSCRIBE_PALLETIZING_OPTIONS_TOPIC_NODE_H
#include <string>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <robin_bridge_generated/PalletizingOptions.h>
// BehaviorTree.CPP
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_ros/bt_topic_sub_node.h>


namespace sysdesign { namespace bt {


class SubscribePalletizingOptions : public BT::RosTopicSubscriberNode<robin_bridge_generated::PalletizingOptions> {
private:
  const std::string LOGNAME = "SubscribePalletizingOptions";

protected:

  void topicCallback(const robin_bridge_generated::PalletizingOptions::ConstPtr &message) override
  {
    // tool
    setOutput("tool_name", message->tool_name);
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "tool_name: " << message->tool_name);

    // object
    setOutput("object_name", message->object_name);
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "object_name: " << message->object_name);
    setOutput("object_weight", message->object_weight);
    ROS_DEBUG_NAMED(LOGNAME, "object_weight: %.1fKg", message->object_weight);

    // pattern manager
    setOutput("pattern_file", message->pattern_file);
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "pattern_file: " << message->pattern_file);
    setOutput("transform_name", message->transform_name);
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "transform_name: " << message->transform_name);

    node_status = BT::NodeStatus::SUCCESS;
  }

public:

  SubscribePalletizingOptions(ros::NodeHandle &node, const std::string &name, const BT::NodeConfiguration &config)
  : BT::RosTopicSubscriberNode<robin_bridge_generated::PalletizingOptions>(node, name, config) { }

  static BT::PortsList providedPorts()
  {
    return
    {
      BT::OutputPort<std::string>("tool_name"),
      BT::OutputPort<std::string>("object_name"),
      BT::OutputPort<double>("object_weight"),
      BT::OutputPort<std::string>("pattern_file"),
      BT::OutputPort<std::string>("transform_name"),
    };
  }

};

} }  // namespace
#endif
