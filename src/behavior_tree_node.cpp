#include <string>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
// BehaviorTree.CPP
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
//
#include <robin_behavior_tree//behavior_tree/CommonActionNode.h>


template<class T>
static void registerRobinAction(BT::BehaviorTreeFactory &factory, const std::string &name, ros::NodeHandle &node_handle)
{
  BT::NodeBuilder builder = [&node_handle](const std::string &name, BT::NodeConfiguration config)
  {
    return std::make_unique<T>(node_handle, name, config);
  };

  factory.registerBuilder<T>(name, builder);
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "behavior_tree_node");

  // Node
  ros::NodeHandle node("~");

  // Parameters
  double freq = node.param("frequency", 1.0);

  std::string tree_path;
  if (!node.getParam("tree_path", tree_path))
  {
    std::string param_name = node.resolveName("tree_path");
    ROS_FATAL("Failed to retrieve '%s' parameter.", param_name.c_str());
    return 1;
  }

  /* BEHAVIOR TREE */
  BT::BehaviorTreeFactory bt_factory;
  registerRobinAction<sysdesign::bt::CommonActionNode>(bt_factory, "CommonAction", node);

  auto tree = bt_factory.createTreeFromFile(tree_path);

  /* LOGGER */
  BT::StdCoutLogger logger(tree);

  /* FILE LOGGER */
  // BT::FileLogger file_logger(tree, "bt_trace.fbl");

  /* ROSOUT LOGGER */
  // BT::RosoutLogger rosout_logger(tree.rootNode());

  /* GROOT */
  BT::PublisherZMQ publisher_zmq(tree);

  // Loop
  ros::Rate rate(freq);
  while (ros::ok() && (tree.tickRoot() != BT::NodeStatus::SUCCESS))
  {
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
