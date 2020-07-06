from rclpy.node import Node
from conditions.condition_publisher import ConditionPublisher, ConditionEnum
from typing import Dict, List
import os
from rclpy.task import Future

class ConditionPublisherNode(Node):
    def __init__(self, name : str):
        super().__init__('condition_publisher_' + str(os.getpid()))
        self.__condpub = ConditionPublisher(self, name)

    def publish(self, condition : ConditionEnum, message : str=""):
        self.__condpub.publish(condition, message)

    def add_single_equality_check(self, topic_name : str, topic_type : str, expected_values : Dict) -> Future:
        return self.__condpub.add_single_equality_check(topic_name, topic_type, expected_values)

    def add_allof_equality_check(self, topic_names_and_expected_values : List[Dict]) -> Future:
        return self.__condpub.add_allof_equality_check(topic_names_and_expected_values)