import rclpy
import sys
import os
import argparse
import yaml
from rclpy.task import Future
from conditions.message_equality_tester_node import MessageEqualityTesterNode

def nonnegative_int(inval):
    ret = int(inval)
    if ret < 0:
        # The error message here gets completely swallowed by argparse
        raise ValueError('Value must be positive or zero')
    return ret

def main():
   
    parser = argparse.ArgumentParser(description=
        'CLI tool for checking message equality')
    parser.add_argument('--name', required=True,
        help='The name of the message to listen for')
    parser.add_argument('--type', required=True,
        help='The type of the message')
    parser.add_argument('--expected', required=True,
        help='The expected values (YAML format)')
    parser.add_argument('--timeout', default=None, type=nonnegative_int,
        help='The amount of time to wait for a message')

    args = parser.parse_args()

    if len(sys.argv)==1:
        parser.print_help(sys.stdout)
        sys.exit(0)
    
    values_dictionary = yaml.safe_load(args.expected)
    if not isinstance(values_dictionary, dict):
        return 'The passed value needs to be a dictionary in YAML format'

    rclpy.init()

    future = Future()

    node = MessageEqualityTesterNode(args.name, args.type, values_dictionary, future)

    rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
    
    if not future.done():
        node.get_logger().error(
            "Timeout after " + str(args.timeout) + 
            " seccond: No message on \"" + args.name + "\" topic" )

    node.destroy_node()

    rclpy.shutdown()