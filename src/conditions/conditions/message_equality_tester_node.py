from rclpy.node import Node
from rclpy.task import Future
from conditions.message_equality_tester import MessageEqualityTester, default_callback, MultiMessageEqualityTester, EqualityType, TopicAndValuesPair
from typing import Dict, Any, List
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
        self.get_logger().info("Message received and equality tested:quality tested: {}".format(str(val)))
        if val is False:
            self.get_logger().info("\tExpected: {}".format(str(expected_values)))
            self.get_logger().info("\tActual: {}".format(str(actual_msg)))

        self.__future_result.set_result(val)

class MultiMessageEqualityTesterNode(Node):

    def __init__(self, topic_and_expected_values_pairs : List[TopicAndValuesPair],
        eqaulity_type : EqualityType, future_result : Future):

        super().__init__('multi_message_equality_tester' + str(os.getpid()))

        self.__tester = MultiMessageEqualityTester(self, topic_and_expected_values_pairs,
            eqaulity_type, self.__callback)

        self.__future_result = future_result
    
    def __callback(self, val : bool,
        message_checks : Dict[str,bool], comparison : TopicAndValuesPair,
        equality_type : EqualityType):

        num_checked = len(message_checks)
        num_to_check = len(comparison)

        self.get_logger().info("Message received and equality tested ({}/{}): {}".format(num_checked, num_to_check, str(val)))
        if val is False:
            self.get_logger().info("\tWith:")
            self.get_logger().info("\t" + str(equality_type))
            self.get_logger().info("\tThe following messages are equal to their comparison:")
            self.get_logger().info("\t" + str(message_checks))
            self.get_logger().info("\tComared against:")
            self.get_logger().info("\t" + str(comparison))
        
        if num_checked is num_to_check:
            self.__future_result.set_result(val)
