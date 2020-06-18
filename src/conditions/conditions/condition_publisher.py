import rclpy
from rclpy.node import Node
import os
import typing
from typing import List, Optional, Dict
from condition_msgs.msg import Condition
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from enum import IntEnum
from conditions.message_equality_tester import MessageEqualityTester
import sys
import os
import argparse
import yaml
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

class ConditionPublisherNode(Node):
    def __init__(self, name : str):
        super().__init__('condition_publisher_' + str(os.getpid()))
        self.__condpub = ConditionPublisher(self, name)

    def publish(self, condition : ConditionEnum, message : str=""):
        self.__condpub.publish(condition, message)

    def add_single_equality_check(self, topic_name : str, topic_type : str, expected_values : Dict) -> Future:
        return self.__condpub.add_single_equality_check(topic_name, topic_type, expected_values)

def nonnegative_int(inval):
    ret = int(inval)
    if ret < 0:
        # The error message here gets completely swallowed by argparse
        raise ValueError('Value must be positive or zero')
    return ret


def main():
       
    parser = argparse.ArgumentParser(description= 'CLI tool for publishing condition messages')

    parser.add_argument('--name', required=True, help='The name of the Condition')
    parser.add_argument('--timeout', default=None, type=nonnegative_int,
        help='The amount of time to wait for a message')

    group = parser.add_mutually_exclusive_group(required=True)
    
    group.add_argument('--value', help='Publish condition with a value',
    choices=['ACTIVE', 'INACTIVE', 'SHELVED', 'SUPPRESSED', 'UNKNOWN'])

    group.add_argument('--single_topic_equality_predicate',
        help='Publish condition as ACTIVE if the last message on a topic is equal to a value, otherwise \
        it will be INACTIVE if not equal, or UNKNOWN if no message is available on the topic.\
        YAML format, \"{topic: <topic_name>, type: <type>, expected_value: <expected_value_as_yaml>}\"\
        (e.g \"{topic: chatter, type: std_msgs/msg/String, expected_value: {data: hello}}\" )')

    group.add_argument('--multi_topic_allof_equality_predicate',
        help='Publish condition as ACTIVE if the last message on all given topics is equal to the associated value, otherwise \
        it will be INACTIVE if not equal, or UNKNOWN if no message is available on the topic.\
        YAML format, \"{ topics: [{topic: <topic_name>, type: <type>, expected_value: <expected_value_as_yaml>}, [{...}]}\"\
        (e.g \""{ topics: [ {topic: chatter, type: std_msgs/msg/String, expected_value: {data: hello} } , {topic: chatter2, type: std_msgs/msg/String, expected_value: {data: hello2}} ]}"\" )')

    group.add_argument('--multi_topic_anyof_equality_predicate',
        help='Publish condition as ACTIVE if the last message on any given topics is equal to the associated value, otherwise \
        it will be INACTIVE if not equal, or UNKNOWN if no message is available on the topic.\
        YAML format, \"{ topics: [{topic: <topic_name>, type: <type>, expected_value: <expected_value_as_yaml>}, [{...}]}\"\
        (e.g \""{ topics: [ {topic: chatter, type: std_msgs/msg/String, expected_value: {data: hello} } , {topic: chatter2, type: std_msgs/msg/String, expected_value: {data: hello2}} ]}"\" )')

    group.add_argument('--multi_topic_exclusiveany_equality_predicate',
        help='Publish condition as ACTIVE if the last message on one and only one given topics is equal to the associated value, otherwise \
        it will be INACTIVE if not equal, or UNKNOWN if no message is available on the topic.\
        YAML format, \"{ topics: [{topic: <topic_name>, type: <type>, expected_value: <expected_value_as_yaml>}, [{...}]}\"\
        (e.g \""{ topics: [ {topic: chatter, type: std_msgs/msg/String, expected_value: {data: hello} } , {topic: chatter2, type: std_msgs/msg/String, expected_value: {data: hello2}} ]}"\" )')


    args = parser.parse_args()

    if len(sys.argv)==1:
        parser.print_help(sys.stdout)
        sys.exit(0)

    if args.value:
        print("coming soon...")

    if args.single_topic_equality_predicate:
        single_topic(args)

    if args.multi_topic_allof_equality_predicate:
        multi_topic_allof(args)

    if args.multi_topic_anyof_equality_predicate:
        print("coming soon...")

    if args.multi_topic_exclusiveany_equality_predicate:
        print("coming soon...")

def single_topic(args):
    values_dictionary = yaml.safe_load(args.single_topic_equality_predicate)
    if not isinstance(values_dictionary, dict):
        print('The passed value needs to be a dictionary in YAML format')
        exit(1)

    try:
        topic = values_dictionary['topic']
    except:
        print("\"topic\" key required in YAML arguemnt. Use -h for help")
        exit(1)

    try:
        msgtype = values_dictionary['type']
    except:
        print("\"type\" key required in YAML arguemnt. Use -h for help")
        exit(1)

    try:
        expected_values_dict = values_dictionary['expected_value']
    except:
        print("\"expected_value\" key required in YAML arguemnt. Use -h for help")
        exit(1)

    rclpy.init()

    node = ConditionPublisherNode(args.name)
    fut = node.add_single_equality_check(topic, msgtype, expected_values_dict)

    node.get_logger().info("Started node \"condition_publisher_" + str(os.getpid()) +
        "\" with publisher for \"" + args.name + "\" and subcription for \"" +
        topic + "\"")

    #rclpy.spin_until_future_complete(node, fut, timeout_sec=args.timeout)
    rclpy.spin(node)

    # if not fut.done():
    #     node.get_logger().error(
    #         "Timeout after " + str(args.timeout) + 
    #         " seccond: No message on \"" + topic + "\" topic" )

    node.destroy_node()
    rclpy.shutdown()



def multi_topic_allof(args):
    values_dictionary = yaml.safe_load(args.multi_topic_allof_equality_predicate)
    if not isinstance(values_dictionary, dict):
        print('The passed value needs to be a dictionary in YAML format')
        exit(1)

    try:
        topics_array = values_dictionary['topics']
    except:
        print("\"topics\" key required in YAML arguemnt. Use -h for help")
        exit(1)

    for values_dictionary in topics_array:
        try:
            topic = values_dictionary['topic']
            print("Subscribing to \"" + topic + "\"")
        except:
            print("\"topic\" key required in YAML arguemnt. Use -h for help")
            exit(1)

        try:
            msgtype = values_dictionary['type']
            print("Of type \"" + msgtype + "\"")
        except:
            print("\"type\" key required in YAML arguemnt. Use -h for help")
            exit(1)
            
        try:
            expected_values_dict = values_dictionary['expected_value']
            print("And expecting the values \"" + str(expected_values_dict) + "\"\n")

        except:
            print("\"expected_value\" key required in YAML arguemnt. Use -h for help")
            exit(1)
    
    print("Checking all of them equal the expected value, \
        and publishing the result to topic \"" + args.name + "\"")