import rclpy
from rclpy.node import Node
import os
import typing
from typing import List, Optional, Dict
from condition_msgs.msg import Condition
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from enum import IntEnum
from conditions.message_equality_tester import MessageEqualityTester, MultiMessageEqualityTester, EqualityType
from rclpy.task import Future
from conditions.message_equality_tester_node import TopicAndValuesPair

class ConditionEnum(IntEnum):
    ACTIVE=0
    INACTIVE=1
    SHELVED=2
    SUPPRESSED=3
    UNKNOWN=4

class ConditionPublisher:
    """
    This class is constructed with a ros node, which it adds a publisher to.
    """

    def __init__(self, node : Node, condition_name : str):

        latching_qos = QoSProfile(depth=1, history=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)

        self.__node = node
        self.__pub = node.create_publisher(Condition, condition_name, qos_profile=latching_qos)
        self.publish(ConditionEnum.UNKNOWN, "No message has been received")

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

    def add_allof_equality_check(self, topic_names_and_expected_values : List[Dict]):
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