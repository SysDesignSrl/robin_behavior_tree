# Robin - BehaviorTree.CPP

## Description
This package implements a common action node useful to send custom commands to
external devices and monitoring the execution.

## Nodes
behavior_tree_node \
  _The program._

### Published Topics
/robin/\<device\>/command ([robin_bridge_generated/CommonCommand](https://github.com/SysDesignSrl/robin_bridge_generated/blob/main/msg/CommonCommand.msg)) \
  _The command sent to the device._

### Subscribed Topics
/robin/\<device\>/feedback ([robin_bridge_generated/CommonFeedback](https://github.com/SysDesignSrl/robin_bridge_generated/blob/main/msg/CommonFeedback.msg)) \
  _The feedback received from the device._

### Parameters
TODO
