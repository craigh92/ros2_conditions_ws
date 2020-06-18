# Conditions Package

Contains classes and functions for working with Condition Messages. A Condition is a high level state of your system. It could be a super-state of multiple parts of the system, or something more complex such as a intersection of a robot and a predefined volume.

Condition Publishers subscribe to one or more topics on the network, perform some predicate check on message recieved on these topics, and then publish a Condition as a result.

This package offers a `ConditionPublisherNode` that works out of the box, and a `ConditionPublisher` class that can be used to add condition publishing abilities to existing nodes.

A Condition Publisher has one or more subscriptions to topics on the network (of any type), a predicate check, and a publisher that publishes Condition messages. On receit of a new message on any of the subscribed topics, the Condition Publisher will perform the predicate check and publish a new Condition message. When multiple topics are subscribed to, it will publish Condition.UNKNOWN untill at least one message on each topic has been received.

#### Built-in Available Predicate Checks
| Type | Description |
| ---- | ------------|
| single_topic_equality | Subscribe to a single topic, and on receit of new messages publish a condition based on an equality check with predefined data. The data to compare to is given in the form of a python `Dict` of key-value pairs, or as a YAML string (in the case of the CLI tool). If the data matches, then the condition will be `ACTIVE`. | 
| multi_topic_all_of_equality | Similar to `single_topic_equality` but for multiple topics. The data to compare to is given in the form of a python `Dict` of the topics, and the expected value for messages on that topic (also as a python `Dict`), or as YAML string (in the case of the CLI tool). If all of the messages match then the condition will be `ACTIVE`. It will publish `UNKNOWN` untill at least one message on each topic has been received. |
| multi_topic_any_of_equality | Subscribe to multiple topics and publish the Condition as `ACTIVE` if any of the messages match the associated data to compare to. |
| multi_topic_ex_any_of_equality | Similar to `multi_topic_any_of_equality` but the Condition will only be ACTIVE if one and only one of the messages match the associated data to compare to. |

#### Python API

Example:
```python

class MyNode(Node):
  def __init__(self):
      super().__init__('my_node')
      self.__condpub = ConditionPublisher() TODO

```

#### CLI Tool

TODO
