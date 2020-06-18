import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from conditions.message_equality_tester_node import MessageEqualityTesterNode
import pytest
from rclpy.task import Future
from rclpy.executors import MultiThreadedExecutor
from typing import Dict
from conditions.minimal_latching_publisher import MinimalPublisher
import time

def start_minimal_publisher(function):
    """
    Decorator for a function that has arguments (future_message : Future, exe : MultiThreadedExecutor)
    Starts a minimal publisher before executing the function, and tears it down after the function
    has finished
    """

    def decorated_function():
        # SETUP 

        minimal_publisher = None
        future_message = Future()

        print("Setup fixture")
        rclpy.init()
        minimal_publisher = MinimalPublisher()
        exe = MultiThreadedExecutor()
        exe.add_node(minimal_publisher)

        #TEST

        result = function(future_message, exe)

        #TEARDOWN

        print("Teardown fixture")
        minimal_publisher.destroy_node()
        rclpy.shutdown()
        time.sleep(0.2)

        return result

    return decorated_function

def start_a_message_equality_tester_node_and_get_result(future_result : Future, exe : MultiThreadedExecutor, expected_data : Dict):

    node = MessageEqualityTesterNode('topic', 'std_msgs/msg/String', expected_data, future_result)
    
    exe.add_node(node)

    exe.spin_until_future_complete(future_result)

    node.destroy_node()

    return future_result.result()

#BEGIN TESTS

def test_msg_is_equal():
    
    @start_minimal_publisher
    def test(future_message, exe):
        return start_a_message_equality_tester_node_and_get_result(future_message, exe, {'data' : 'Hello World: 0'})
    
    result = test()
    assert result is True

def test_msg_is_not_equal():
    
    @start_minimal_publisher
    def test(future_message, exe):
        return start_a_message_equality_tester_node_and_get_result(future_message, exe, {'data' : 'Hello Worldd: 0'})
    
    result = test()
    assert result is False