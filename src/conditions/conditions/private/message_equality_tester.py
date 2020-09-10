from typing import Callable, Any, Dict, List
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import set_message_fields
from rclpy.node import Node
from enum import Enum

def default_callback(val : bool, actual_msg : Dict, expected_values : Dict):
    print("Message received and equality tested: " + str(val))
    if val is False:
        print("\tExpected:")
        print("\t" + str(expected_values))
        print("\tGot:")
        print("\t" + str(actual_msg))

class MessageEqualityTester:
    """
    Constructed with a ros Node, a topic name and type, an expected value, and a callback.
    Adds a subscription to the node passed in, that tests if the received messages are equal to
    the expected values, and then calls the callback with the result of the equality check. 
    """

    def __init__(self, node : Node, topic_name: str, message_type : str,
        expected_values : Dict[str,str], callback : Callable[[bool, Dict[str, str], Dict[str,str]],None] = default_callback):
        
        self.__callback = callback
        self.__expected_values = expected_values
        self.__message_type = message_type
        self.__node = node

        latching_qos = QoSProfile(depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        
        node.get_logger().info("Subscribing to \"{}\" of type \"{}\"".format(topic_name, message_type))
        
        node.get_logger().info("Expecting to recieve message with value \"{}\"".format(str(expected_values)))

        self.__sub = node.create_subscription(get_message(message_type), topic_name, self.__sub_callback,
            qos_profile=latching_qos)

    def __sub_callback(self, actual_msg):

        # Test the equality:

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

        # Call the callback with the result

        self.__callback(equal, actual_msg, expected_msg)


class EqualityType(Enum):
    ALLOF=0
    ANYOF=1
    EXANY=2

class TopicAndValuesPair:
    def __init__(self, topic_name : str, topic_type : str, values : Dict[str, str]):
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.values = values


def default_multi_message_callback(val : bool, message_checks : Dict[str,bool], comparison : List[TopicAndValuesPair], equality_type : EqualityType):
    print("Messages received and equality tested {}".format(str(val)))
    if val is False:
        print("\tWith:")
        print("\t" + str(equality_type))
        print("\tThe following messages are equal to their comparison:")
        print("\t" + str(message_checks))

class MultiMessageEqualityTester:
    """
    Subscribe to multiple topics, and store the last message received on each topic. On receit of each new
    message perform a predicate check on the store of messages and then call the given callback with
    the result.
    """

    __message_store = {} #Store the last message recievd on each topic. key = topic name, val = recieved message is equal to expected
    __testers = []

    def __init__(self, node : Node, topic_and_expected_values_pairs : List[TopicAndValuesPair],
        eqaulity_type : EqualityType,
        callback : Callable[[bool, Dict[str,bool], List[TopicAndValuesPair], EqualityType],None] = default_multi_message_callback):
        
        self.__equality_type = eqaulity_type
        self.__callback = callback
        self.__topic_and_expected_values_pairs = topic_and_expected_values_pairs
    
        node.get_logger().info("Setting up subscriptions")

        for topic_and_expected_values_pair in topic_and_expected_values_pairs:
            
            #Create MessageEqualityTester that store the result in __message_store when a new message
            #is received, and then perform the predicate check (which calls the callback)
 
            def cb(val : bool, actual_msg : Dict, expected_values : Dict, topic_and_expected_values_pair=topic_and_expected_values_pair):
                
                #Store the value of that message
                self.__message_store[topic_and_expected_values_pair.topic_name] = val
                
                node.get_logger().info("Message received ({}/{})".format(len(self.__message_store), len(topic_and_expected_values_pairs)))
                
                if(len(self.__message_store) == len(topic_and_expected_values_pairs)):
                    node.get_logger().info("All messages received, performing check")
                    self.__perform_check()

            tester = MessageEqualityTester(node, topic_and_expected_values_pair.topic_name,
                topic_and_expected_values_pair.topic_type,
                topic_and_expected_values_pair.values, cb)

            self.__testers.append(tester)

        node.get_logger().info("Waiting for at least one message on each topic...")

    def __perform_check(self):
        """
        Gets called when at least one message on each topic has arrived.
        Iterate over the message_store, and call the callback with the result of the predicate
        """
         
        if self.__equality_type is EqualityType.ALLOF:

            alltrue = True
            for topic in self.__message_store:
                is_equal = self.__message_store[topic]
                if is_equal is not True:
                    alltrue = False

            self.__callback(alltrue, self.__message_store, self.__topic_and_expected_values_pairs, self.__equality_type)

        if self.__equality_type is EqualityType.ANYOF:
            anyTrue = False
            for topic in self.__message_store:
                is_equal = self.__message_store[topic]
                if is_equal is True:
                    anyTrue = True

            self.__callback(anyTrue, self.__message_store, self.__topic_and_expected_values_pairs, self.__equality_type)

        if self.__equality_type is EqualityType.EXANY:
            oneTrue = False
            for topic in self.__message_store:
                is_equal = self.__message_store[topic]
                if is_equal is True and oneTrue is False:
                    oneTrue = True
                if is_equal is True and oneTrue is True:
                    oneTrue = False

            self.__callback(oneTrue, self.__message_store, self.__topic_and_expected_values_pairs, self.__equality_type)
