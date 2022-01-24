Monitor Node {#monitored-node-design}
===========

This is the design document for the `monitored_node` package.

# Purpose / Use cases

When running Autoware, there is a need to ensure all nodes have been given sufficient resources and are able to run at the intended frequency.
Any processing pipeline involving multiple nodes should also adhere to a latency quota overall to ensure successful operation.
Hence, a certain monitoring functionality is required to detect when nodes are performing below tolerance and react appropriately.

# Design

The `monitored_node` package provides an API for building ROS2 nodes with builtin callback time monitoring functionalities.

A monitored node will monitor its own performance using the ROS timer API and log messages to the ROS2 logger and to a diagnostic topic when a time limit is exceeded.
The diagnostic topic is then monitored using a dedicated error monitor node.
This dedicated node is in charge of detecting failures and taking appropriate action.

Each monitored publishing node shall, alongside the main topic, publish the intended publishing interval of the topic.
The monitored subscriber nodes can use this information to ensure their subscriber callback is invoked at the intended rate.
The subscriber callback should take less time than the minimum message interval in order to keep up with the rest of the system.
This condition can be checked automatically.
A further time limit can be imposed onto the subscriber callback by the programmer.
This is to deal with situations where the end-to-end latency in a processing chain is limited and each step in the chain gets a quota.

## Assumptions / Known limits

Assumptions:

- ROS timers are accurate.
- The DDS implementation suffers minimum delay

## Inputs / Outputs / API

Instead of inheriting from the `rclcpp::Node` class to build the nodes, the user would use:

```cpp
class MyNode : public MonitoredNode {
  MyNode (const rclcpp::NodeOptions & options)
    : MonitoredNode("listener", options)
  {
    ...
  }
}
```

Publishers and subscribers are created using the `create_monitored_<>` API.
`MonitoredSubscription` and `MonitoredPublisher` are wrappers around the ROS base classes to implement timing checks transparently.
A `MonitoredSubscription` object contains information about the intended publishing interval of the subscribed topic.
This information can be used to determine the intended publishing interval of the publishers in the current node.
A filter node, for example, subscribes to a incoming topic and publishes on the outgoing topic.
The intended rate of the outgoing topic depends on the rate of the incoming topic.

```cpp
MonitoredSubscription::SharedPtr m_sub = create_monitored_subscription<MessageTypeT>("TOPIC",
                                                                                     QoS,
                                                                                     max_callback_time_ms);
rclcpp::Publisher::SharedPtr m_pub = create_monitored_publisher<MessageTypeT>("TOPIC",
                                                                              QoS,
                                                                              m_sub->get_min_interval_future(),
                                                                              m_sub->get_max_interval_future());
```

Instead of specifying the intervals and delays in the cpp API, the user can also override them from ROS2 parameters in the launch files.
The parameter names are:

```
<topic_name>.min_publish_interval_ms
<topic_name>.max_publish_interval_ms
<topic_name>.max_callback_duration_ms
```

Publishing intervals should be set on the publisher side and callback duration set on the subscriber side.

## Inner-workings / Algorithms

### Detecting Anomalies

1. Monitor subscription callback frequency:

   At the start of a callback, a timer is started.
   The timer is set to expire at max_interval specified in the API.
   If a second callback comes in before the timer expires, the elapsed time on the timer is checked.
   If a node is operating normally, the elapsed time should be larger than the min_interval.
   If the timer expires before a second callback occurs or the min_interval check fails, a handler is triggered.

2. Monitor subscription callback duration: same method applies.

   A timer is started before the callback is invoked, and reset at the end of the callback.
   The timer is set to expire at max_duration specified in the API.

### Communication

Monitored nodes communicate with an external error monitor via a single diagnostic topic.
Each time an event happens, eg. callback_started, callback_ended, a message containing the event name and timestamp is sent to the topic.
The monitor node analyses this stream of events and output a signal indicating the vehicle's capability.
The error monitor node can be run on dedicated safety-hardened hardware to ensure its correct operation.

### Interval Propagation

The min/max_interval_ms members of the monitored subscriber are held as futures.
The value is only set after the relevant information is received from an upstream monitored node.
Monitored publishers created with arguments defined as futures which values just got set will in turn announce those values to their subscribers.

Setting the futures' values should always happen before the first message is published from the current monitored node when the interval values are propagated to downstream monitored nodes via the publisher.
Otherwise, the monitoring of callbacks stays inactive until the intervals are set; with the callback themselves being processed normally.

## Error detection and handling

Safety monitor functionality wraps around a normal rclcpp node and should not interfere with the normal error handling mechanism of the node.

# Security considerations

- A dedicated error monitor node is a high value target as it ensures the normal running of the whole system and thus prevents certain types of attacks.
  Hence, the usual steps should be taken to secure Linux and ROS in order to prevent attacks.
  The safety monitor should be run on dedicated safety hardware on a high value deployment.

- The safety monitoring functionality relies on the ROS2 API to work.
  Therefore, usual practices to secure ROS2 and Linux apply.

# Future extensions / Unimplemented parts

See related issues for overall plans.

# Related issues

- [#821](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/821) - Detect when nodes' incoming messages are skipped
- [#1233](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1233) - Autoware monitoring system: Implement Autoware Error Monitor
