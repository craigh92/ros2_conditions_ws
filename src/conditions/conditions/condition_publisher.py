import rclpy
from rclpy.node import Node
import os
import typing
from typing import List, Optional, Dict
from condition_msgs.msg import Condition
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from enum import IntEnum
from conditions.message_equality_tester import MessageEqualityTester

from rclpy.task import Future

class ConditionEnum(IntEnum):
    ACTIVE=0
    INACTIVE=1
    SHELVED=2
    SUPPRESSED=3
    UNKNOWN=4

class ConditionPublisher:
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

    __cache = {}
    def add_allof_equality_check(self, topic_names_and_expected_values : List[Dict]):
        """
        Subscribe to multiple topics, and publish a Condition message on receit of a new message. The Condition
        will be active if the last message published on all of the topics is equal to the associated expected values.
        """

        future = Future()

        for value_dict in topic_names_and_expected_values:
            topic_name = value_dict['topic']
            topic_type = value_dict['type']
            expected_values_dict = value_dict['expected_values']
        
            def equality_check_cb(val : bool, actual_msg : Dict, expected_values : Dict):
                self.__cache[topic_name] = val
                allTrue = True
                for key in self.__cache:
                    if self.__cache[key] is False:
                        allTrue = False
                if allTrue:
                    self.publish(ConditionEnum.ACTIVE)
                else:
                    message = topic_name + " value is not equal to " + str(expected_values) + " (actual value: " + str(actual_msg) + ")"
                    self.publish(ConditionEnum.INACTIVE, message)               
                future.set_result(allTrue)

            MessageEqualityTester(self.__node, topic_name, topic_type, expected_values_dict ,equality_check_cb)
            
        return future