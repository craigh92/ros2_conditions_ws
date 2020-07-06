import rclpy
import sys
import os
import argparse
import yaml
from rclpy.task import Future
from conditions.message_equality_tester_node import MultiMessageEqualityTesterNode, TopicAndValuesPair, EqualityType

"""
Example use:
ros2 run conditions multi_message_equality_tester --expected_msgs "{topics: [ {topic: chatter, type: std_msgs/msg/String, expected_value: {data: hello} } , {topic: chatter2, type: std_msgs/msg/String, expected_value: {data: hello2}} ]}"
"""

def nonnegative_int(inval):
    ret = int(inval)
    if ret < 0:
        # The error message here gets completely swallowed by argparse
        raise ValueError('Value must be positive or zero')
    return ret

def main():
   
    parser = argparse.ArgumentParser(description=
        'CLI tool for checking message equality')
    
    parser.add_argument('--expected_msgs', required=True,
        help='The name of the messages to listen for with the type and expected value\
        YAML format, \"{ topics: [{topic: <topic_name>, type: <type>, expected_value: <expected_value_as_yaml>}, [{...}]}\"\
        (e.g \""{ topics: [ {topic: chatter, type: std_msgs/msg/String, expected_value: {data: hello} } , {topic: chatter2, type: std_msgs/msg/String, expected_value: {data: hello2}} ]}"\" )')
    
    parser.add_argument('--timeout', default=None, type=nonnegative_int,
        help='The amount of time to wait for a message')

    args = parser.parse_args()

    if len(sys.argv)==1:
        parser.print_help(sys.stdout)
        sys.exit(0)
    
    print("")

    values_dictionary = yaml.safe_load(args.expected_msgs)
    if not isinstance(values_dictionary, dict):
        print('The passed value needs to be a dictionary in YAML format')
        exit(1)

    rclpy.init()

    try:
        topics_array = values_dictionary['topics']
    except:
        print("\"topics\" key required in YAML arguemnt. Use -h for help")
        exit(1)

    topics_and_expected_values = []
    for values_dictionary in topics_array:
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

        topics_and_expected_values.append(TopicAndValuesPair(topic, msgtype, expected_values_dict))
    
    future = Future()
    node = MultiMessageEqualityTesterNode(topics_and_expected_values, EqualityType.ALLOF, future)

    rclpy.spin_until_future_complete(node, future)

    rclpy.shutdown()