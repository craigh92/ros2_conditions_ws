from typing import Callable, Any, Dict
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import set_message_fields
from rclpy.node import Node

#TODO open a new PR in ros2/roscli and add this as a new Verb: ros2 topic expect <args>

def default_callback(val : bool, actual_msg : Dict, expected_values : Dict):
    print("Equality Tested: " + str(val))
    if val is False:
        print("Expected:")
        print(str(expected_values))
        print("Got:")
        print(str(actual_msg))

class MessageEqualityTester:
    """
    Constructed with a ros Node, a topic name and type, an expected value, and a callback.
    Adds a subscription to the node passed in, that tests if the received messages are equal to
    the expected values, and then calls the callback with the result of the equality check. 
    """

    def __init__(self, node : Node, topic_name: str, message_type : str,
        expected_values : Dict, callback : Callable[[bool, Any, Dict],None] = default_callback):
        
        self.callback = callback
        self.__expected_values = expected_values
        self.__message_type = message_type
        self.__node = node

        latching_qos = QoSProfile(depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        
        node.get_logger().info("Subscribing to \"{}\" of type \"{}\"".format(topic_name, message_type))
        
        self.__sub = node.create_subscription(get_message(message_type), topic_name, self.__sub_callback,
            qos_profile=latching_qos)

    def __sub_callback(self, actual_msg):

        expected_msg = get_message(self.__message_type)()

        equal = True
        try:
            set_message_fields(expected_msg, self.__expected_values)
        except Exception as e:
            self.__node.get_logger().error('Failed to populate field: {0}'.format(e))
            raise e

        for field in expected_msg.get_fields_and_field_types():
            expected = str(getattr(expected_msg, field))
            actual = str(getattr(actual_msg, field))
            if expected != actual:
                equal = False

        self.callback(equal, actual_msg, expected_msg)
