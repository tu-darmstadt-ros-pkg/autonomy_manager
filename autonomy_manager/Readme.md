# Autonomy Manager

The Autonomy Manager ROS package is designed to facilitate access to the current autonomy mode of a robot. It achieves
this by subscribing to specified topics and identifying the autonomy mode based on the nodes publishing to these topics.

## Overview

The Autonomy Manager package enables the classification of autonomy modes into one of the following categories:

- Teleoperated
- Semi-autonomous
- Autonomous
- Unknown

It accomplishes this by monitoring designated topics and analyzing the messages sent by known nodes associated with
different autonomy modes.

## Configuration

The Autonomy Manager requires configuration parameters to function properly. These parameters are specified in the ROS
parameter server. Here is an example of the configuration:

```yaml
autonomy_manager:
  ros__parameters:
    twist_topic: "cmd_vel"
    twist_nodes: [ "test_1", "test_2" ] # nodes that publish to twist_topic
    twist_autonomy_types: [ "TELEOP", "AUTONOMY" ] # the autonomy types of the nodes that publish to twist_topic
```

Explanation of the parameters:

    twist_topic: Specifies the ROS topic to which messages related to motion commands are published.
    twist_nodes: Lists the names of nodes that publish messages to the specified twist_topic.
    twist_autonomy_types: Specifies the autonomy types associated with the nodes publishing to the twist_topic.

# Autonomy Message

The Autonomy Manager publishes an autonomy_mode message to indicate the current autonomy mode. The following constants
are defined for different autonomy modes:

    UNKNOWN: 0
    TELEOPERATED: 1
    AUTONOMOUS: 2
    SEMI_AUTONOMOUS: 3

# Example / Test

To demonstrate the functionality of the Autonomy Manager, an example test file is provided. This test file showcases
several nodes publishing messages on the cmd_vel topic. The Autonomy Manager dynamically adjusts the autonomy mode based
on the autonomy information provided by the sending nodes.

# Usage

Once the package is properly configured and launched, subscribers can listen to the autonomy_mode topic to access the
current autonomy mode of the robot. This information can be utilized by other ROS nodes to adapt their behavior
accordingly.

# Limitations
#### Predefined types
Since the generic_subscriber does not seem to offer the possibility to get the corresponding msg info (which is required to infer the publisher id),
the current implementation is limited to use a set of predefined types.

#### Simultaneous publishers
If Nodes with different autonomy types publish to the same topic at the same time, the Autonomy Manager will set the mode correctly. 
Hence, the mode will change frequently, which might not be the desired behavior.
