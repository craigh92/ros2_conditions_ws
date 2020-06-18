from rclpy.node import Node
from rclpy.task import Future
from conditions.message_equality_tester import MessageEqualityTester, default_callback
from typing import Dict, Any
import os

class MessageEqualityTesterNode(Node):
    """
    Constructed with a Future. the future will be set to True or False on receit of the first message,
    depending if the message matches the expected values
    """
    
    def __init__(self, topic_name: str, message_type : str,
        expected_values : Dict, future_result : Future):
        
        super().__init__('message_equality_tester' + str(os.getpid()))
        
        self.__tester = MessageEqualityTester(self, topic_name, message_type, expected_values,
            self.__callback)

        self.__future_result = future_result

    def __callback(self, val : bool, actual_msg : Dict, expected_values : Dict):
        self.get_logger().info("Message received and equality tested: {}".format(str(val)))
        if val is False:
            self.get_logger().info("Expected: {}".format(str(expected_values)))
            self.get_logger().info("Actual: {}".format(str(actual_msg)))

        self.__future_result.set_result(val)
