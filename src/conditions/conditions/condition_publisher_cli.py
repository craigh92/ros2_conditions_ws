import rclpy
import sys
import os
import argparse
import yaml
from conditions.condition_publisher_node import ConditionPublisherNode
from conditions.message_equality_tester_node import TopicAndValuesPair

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

    # group.add_argument('--multi_topic_anyof_equality_predicate',
    #     help='Publish condition as ACTIVE if the last message on any given topics is equal to the associated value, otherwise \
    #     it will be INACTIVE if not equal, or UNKNOWN if no message is available on the topic.\
    #     YAML format, \"{ topics: [{topic: <topic_name>, type: <type>, expected_value: <expected_value_as_yaml>}, [{...}]}\"\
    #     (e.g \""{ topics: [ {topic: chatter, type: std_msgs/msg/String, expected_value: {data: hello} } , {topic: chatter2, type: std_msgs/msg/String, expected_value: {data: hello2}} ]}"\" )')

    # group.add_argument('--multi_topic_exclusiveany_equality_predicate',
    #     help='Publish condition as ACTIVE if the last message on one and only one given topics is equal to the associated value, otherwise \
    #     it will be INACTIVE if not equal, or UNKNOWN if no message is available on the topic.\
    #     YAML format, \"{ topics: [{topic: <topic_name>, type: <type>, expected_value: <expected_value_as_yaml>}, [{...}]}\"\
    #     (e.g \""{ topics: [ {topic: chatter, type: std_msgs/msg/String, expected_value: {data: hello} } , {topic: chatter2, type: std_msgs/msg/String, expected_value: {data: hello2}} ]}"\" )')


    args = parser.parse_args()

    if len(sys.argv)==1:
        parser.print_help(sys.stdout)
        sys.exit(0)

    if args.value:
        print("coming soon...")

    if args.single_topic_equality_predicate:
        single_topic(args)

    else:
        multi_topic(args)

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



def multi_topic(args):

    values_dictionary = yaml.safe_load(args.multi_topic_allof_equality_predicate)
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
    
    node = ConditionPublisherNode(args.name)
    fut = node.add_allof_equality_check(topics_and_expected_values)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()