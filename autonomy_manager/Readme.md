# Autonomy Manager

The Autonomy Manager ROS package is designed to facilitate access to the current autonomy mode of a robot. It achieves
this by subscribing to specified topics and identifying the autonomy mode based on the nodes publishing to these topics.
If it observes more than one topic, it will select the highest autonomy level among the observed topics.
(Autonomy levels are defined in the `autonomy_msgs` package.)

## Overview

The Autonomy Manager package enables the classification of autonomy modes into one of the following categories:

- Unknown
- Teleoperated
- Semi-autonomous
- Autonomous
- Inactive

It accomplishes this by monitoring designated topics and analyzing the messages sent by known nodes associated with
different autonomy modes.

## Configuration

The Autonomy Manager requires configuration parameters to function properly. These parameters are specified in the ROS
parameter server. Here is an example of the configuration:

```yaml
autonomy_manager:
  ros__parameters:

    stale_mode: "INACTIVE" # the mode that should be set if no new messages arrive (after the topic-specific timeout has passed)

    observed_topics: [ "cmd_vel" ] # topics that autonomy manager should observe

    # per observed topic one configuration
    cmd_vel_nodes: [ "test_1", "test_2" ] # nodes that publish to cmd_vel
    cmd_vel_autonomy_types: [ "TELEOP", "AUTONOMY" ] # the autonomy types of the nodes that publish to cmd_vel
    cmd_vel_timeout: 15.0 # timeout in seconds after which the autonomy manager should invalidate the mode if no new messages arrive
```

# Autonomy Message

The Autonomy Manager publishes an autonomy_mode message to indicate the current autonomy mode. The following constants
are defined for different autonomy modes:

    int8 autonomy_mode
    # Constants (ordered by priority)
    int8 UNKNOWN = 0
    int8 TELEOPERATED = 1
    int8 SEMI_AUTONOMOUS = 2
    int8 AUTONOMOUS = 3
    int8 INACTIVE = 4


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
