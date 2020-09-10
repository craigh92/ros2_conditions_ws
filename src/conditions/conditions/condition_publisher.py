import rclpy
from rclpy.node import Node
import os
import typing
from typing import List, Optional, Dict, Type, Any
from condition_msgs.msg import Condition
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from enum import IntEnum
from conditions.private.message_equality_tester import MessageEqualityTester, MultiMessageEqualityTester, EqualityType
from rclpy.task import Future
from conditions.private.message_equality_tester_node import TopicAndValuesPair

class ConditionEnum(IntEnum):
    ACTIVE=0
    INACTIVE=1
    SHELVED=2
    SUPPRESSED=3
    UNKNOWN=4


#TODO Change TopicAndValuesPair to this
# class PredicateArgs:
#     """
#     The arguments used to check if the last message received on a topic is equal to an expected value
#     """
#     def __init__(self, topic_name: str, topic_type: Type, expected_vale: Dict[str, Any]):
#         self.topic_name: str = topic_name
#         self.topic_type: Type = topic_type
#         self.expected_vale: Dict[str, Any] = expected_vale

class ConditionPublisher:
    """
    This class is constructed with a ros node, which it adds a publisher to.... TODO and subscriptions, cache etc
    This only publishes one condition, which it why it is not a node. Compose multiple of these inside a ros node
    if multiple conditions should be published.
    """

    def __init__(self,
        node: Node,
        condition_name: str,
        *,
        all_of_equality_check: Optional[List[TopicAndValuesPair]] = None,
        any_of_equality_check: Optional[List[TopicAndValuesPair]] = None,
        ex_any_of_equality_check: Optional[List[TopicAndValuesPair]] = None
        #intersection_check: Optional[PointAndVolume]
        ):

        latching_qos = QoSProfile(depth=1, history=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)

        self.__node = node
        self.__pub = node.create_publisher(Condition, condition_name, qos_profile=latching_qos)
        self.publish(ConditionEnum.UNKNOWN, "No message has been received")

        if all_of_equality_check is not None:
            self.add_allof_equality_check(all_of_equality_check)
        if any_of_equality_check is not None:
            raise NotImplementedError()
        if ex_any_of_equality_check is not None:
            raise NotImplementedError()

    def publish(self, condition : ConditionEnum, message : str=""):
        """
        Publish a one off condition message. This is called by the prediacte check subscriptions, but can
        also be called manually if is required.
        """
        self.__node.get_logger().info("Publishing " + str(condition))
        msg = Condition()
        msg.condition = int(condition)
        msg.message = message
        self.__pub.publish(msg)

    def add_single_equality_check(self, topic_name : str, topic_type : str, expected_values : Dict) -> Future:
        """
        Subscribe to a topic, and publish a Condition message on receit of a new message. The Condition
        will be active if the received message equals the expected values.
        """

        future = Future()
        
        def equality_check_cb(val : bool, actual_msg : Dict, expected_values : Dict):
            if val is True:
                self.publish(ConditionEnum.ACTIVE)
            else:
                message = topic_name + " value is not equal to " + str(expected_values) + " (actual value: " + str(actual_msg) + ")"
                self.publish(ConditionEnum.INACTIVE, message)
            future.set_result(val)

        MessageEqualityTester(self.__node, topic_name, topic_type, expected_values,equality_check_cb)
        
        return future

    def add_allof_equality_check(self, topic_names_and_expected_values : List[TopicAndValuesPair]):
        """
        Subscribe to multiple topics, and publish a Condition message on receit of a new message. The Condition
        will be active if the last message published on all of the topics is equal to the associated expected values.
        """

        future = Future()

        def equality_check_cb(val : bool, message_checks : Dict[str,bool], comparison : List[TopicAndValuesPair], equality_type : EqualityType):
            self.__node.get_logger().info("All messages received and equality tested {}".format(str(val)))
            if val is True:
                self.publish(ConditionEnum.ACTIVE)
            else:
                failed = list(filter(lambda key : message_checks[key] is True, message_checks))
                self.publish(ConditionEnum.INACTIVE, "The following checks failed: " + str(failed))

            future.set_result(val)

        MultiMessageEqualityTester(self.__node, topic_names_and_expected_values, EqualityType.ALLOF, equality_check_cb)

        return future