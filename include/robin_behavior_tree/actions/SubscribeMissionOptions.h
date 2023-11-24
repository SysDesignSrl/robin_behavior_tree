#ifndef ROBIN_BEHAVIOR_TREE_SUBSCRIBE_MISSION_OPTIONS_TOPIC_NODE_H
#define ROBIN_BEHAVIOR_TREE_SUBSCRIBE_MISSION_OPTIONS_TOPIC_NODE_H
#include <string>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <robin_bridge_generated/MissionOptions.h>
// BehaviorTree.CPP
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_ros/bt_topic_sub_node.h>


namespace sysdesign { namespace bt {


class SubscribeMissionOptions : public BT::RosTopicSubscriberNode<robin_bridge_generated::MissionOptions> {
private:
  const std::string LOGNAME = "SubscribeMissionOptions";
  robin_bridge_generated::MissionOptions message;

protected:

  void topicCallback(const robin_bridge_generated::MissionOptions::ConstPtr &message) override
  {
    this->message = *message;
    node_status = BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus tick() override
  {
    // skip
    setOutput<bool>("skip", message.skip);
    ROS_DEBUG_NAMED(LOGNAME, "skip: %s", (message.skip) ? "True" : "False");

    // mission
    setOutput<std::string>("mission_type", message.mission_type);
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "mission_type: " << message.mission_type);

    // object
    setOutput<std::string>("object_type", message.object_type);
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "object_type: " << message.object_type);

    // pick
    {
      setOutput<std::string>("object_frame", message.object_pose.header.frame_id);
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "object_frame: " << message.object_pose.header.frame_id);

      double x, y, z;
      x = message.object_pose.pose.position.x;
      y = message.object_pose.pose.position.y;
      z = message.object_pose.pose.position.z;

      double qx, qy, qz, qw;
      qx = message.object_pose.pose.orientation.x;
      qy = message.object_pose.pose.orientation.y;
      qz = message.object_pose.pose.orientation.z;
      qw = message.object_pose.pose.orientation.w;

      char buffer[100];
      std::sprintf(buffer, "%.3f;%.3f;%.3f;%.6f;%.6f;%.6f;%.6f", x, y, z, qx, qy, qz, qw);
      std::string output(buffer);

      setOutput<geometry_msgs::Pose>("object_pose", message.object_pose.pose);
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "object_pose: " << output);
    }

    // place
    {
      setOutput<std::string>("location_frame", message.location_pose.header.frame_id);
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "location_frame: " << message.location_pose.header.frame_id);

      double x, y, z;
      x = message.location_pose.pose.position.x;
      y = message.location_pose.pose.position.y;
      z = message.location_pose.pose.position.z;

      double qx, qy, qz, qw;
      qx = message.location_pose.pose.orientation.x;
      qy = message.location_pose.pose.orientation.y;
      qz = message.location_pose.pose.orientation.z;
      qw = message.location_pose.pose.orientation.w;

      char buffer[100];
      std::sprintf(buffer, "%.3f;%.3f;%.3f;%.6f;%.6f;%.6f;%.6f", x, y, z, qx, qy, qz, qw);
      std::string output(buffer);

      setOutput<geometry_msgs::Pose>("location_pose", message.location_pose.pose);
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "location_pose: " << output);
    }

    return node_status;
  }

public:

  SubscribeMissionOptions(ros::NodeHandle &node, const std::string &name, const BT::NodeConfiguration &config)
  : BT::RosTopicSubscriberNode<robin_bridge_generated::MissionOptions>(node, name, config) { }

  static BT::PortsList providedPorts()
  {
    return
    {
      BT::OutputPort<bool>("skip"),
      BT::OutputPort<std::string>("mission_type"),
      BT::OutputPort<std::string>("object_type"),
      BT::OutputPort<std::string>("object_frame"),
      BT::OutputPort<geometry_msgs::Pose>("object_pose"),
      BT::OutputPort<std::string>("location_frame"),
      BT::OutputPort<geometry_msgs::Pose>("location_pose"),
    };
  }

};

} }  // namespace
#endif