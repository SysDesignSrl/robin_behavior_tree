#include <string>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
// BehaviorTree.CPP
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_ros/bt_topic_sub_node.h>
//
#include <robin_behavior_tree/actions/CommonAction.h>
#include <robin_behavior_tree/actions/SubscribePalletizingOptions.h>


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
  ros::NodeHandle node_ns("");

  // Parameters
  double freq = node.param("frequency", 1.0);

  std::string bt_file;
  if (!node.getParam("bt_file", bt_file))
  {
    std::string param_name = node.resolveName("bt_file");
    ROS_FATAL("Failed to retrieve '%s' parameter.", param_name.c_str());
    return 1;
  }

  /* BEHAVIOR TREE */
  BT::BehaviorTreeFactory bt_factory;

  registerRobinAction<sysdesign::bt::CommonAction>(bt_factory, "CommonAction", node);
  
  BT::RegisterRosTopicSubscriber<sysdesign::bt::SubscribePalletizingOptions>(bt_factory, "SubscribePalletizingOptions", node);


  BT::Tree bt_tree;
  try 
  {
    bt_tree = bt_factory.createTreeFromFile(bt_file);
  }
  catch (const BT::RuntimeError &ex)
  {
    ROS_FATAL("Runtime Error: %s", ex.what());
    return 1;
  }

  /* LOGGER */
  BT::StdCoutLogger logger(bt_tree);

  /* FILE LOGGER */
  // BT::FileLogger file_logger(tree, "bt_trace.fbl");

  /* ROSOUT LOGGER */
  // BT::RosoutLogger rosout_logger(tree.rootNode());

  /* GROOT */
  // BT::PublisherZMQ publisher_zmq(tree);

  // Loop
  ros::Rate rate(freq);
  while (ros::ok() && (bt_tree.tickRoot() != BT::NodeStatus::SUCCESS))
  {
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
