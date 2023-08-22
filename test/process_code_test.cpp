#include <string>
// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <gtest/gtest.h>
// BehaviorTree.CPP
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
//
#include <robin_behavior_tree/actions/mocks/ProcessCode.h>


template<class T>
static void registerRobinAction(BT::BehaviorTreeFactory &factory, const std::string &name, ros::NodeHandle &node_handle)
{
  BT::NodeBuilder builder = [&node_handle](const std::string &name, BT::NodeConfiguration config)
  {
    return std::make_unique<T>(node_handle, name, config);
  };

  factory.registerBuilder<T>(name, builder);
}


class ProcessCodeFixture : public ::testing::Test {
protected:

  ros::NodeHandle node;
  ros::NodeHandle node_ns;

  BT::BehaviorTreeFactory bt_factory;
  BT::Tree bt_tree;

  ProcessCodeFixture() : node("~"), node_ns("")
  {

  }

  void SetUp() override
  {
    registerRobinAction<sysdesign::bt::ProcessCode>(bt_factory, "ProcessCode", node_ns);
  }

};


TEST_F(ProcessCodeFixture, tickRoot)
{
  std::string bt_file;
  ASSERT_TRUE(node.getParam("bt_file", bt_file));
  ASSERT_NO_THROW({
    // bt_tree = bt_factory.createTreeFromFile(bt_file);
    bt_factory.registerBehaviorTreeFromFile(bt_file);
    bt_tree = bt_factory.createTree("TestTree");
  });

  BT::NodeStatus bt_status;

  /* LOGGER */
  BT::StdCoutLogger logger(bt_tree);

  double freq = node.param("frequency", 1.0);
  ros::Rate rate(freq);
  do
  {
    rate.sleep();
    bt_status = bt_tree.tickRoot();
    ros::spinOnce();
  }
  while ((bt_status == BT::NodeStatus::RUNNING) && node.ok());
  ASSERT_EQ(bt_status, BT::NodeStatus::SUCCESS);
}


int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "process_code_test");

  return RUN_ALL_TESTS();
}