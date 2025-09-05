#!/usr/bin/env python3
# Copyright 2024 Moritz Schmidt
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci
import time


class FibonacciActionServer(Node):
    """Simple Fibonacci action server for testing the ActionClient"""
    
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )
        self.get_logger().info('Fibonacci action server started and ready')

    def execute_callback(self, goal_handle):
        """Execute the Fibonacci action"""
        self.get_logger().info(f'Executing goal with order: {goal_handle.request.order}')
        
        order = goal_handle.request.order
        
        if order < 0:
            self.get_logger().warn(f'Invalid order: {order}, aborting goal')
            goal_handle.abort()
            result = Fibonacci.Result()
            return result
            
        # Start with [0, 1] for order >= 1, or [0] for order == 0
        if order == 0:
            sequence = [0]
        else:
            sequence = [0, 1]
        
        # Send feedback periodically
        feedback_msg = Fibonacci.Feedback()
        
        # Generate Fibonacci sequence up to the requested order
        for i in range(2, order + 1):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                goal_handle.canceled()
                result = Fibonacci.Result()
                result.sequence = sequence
                return result
                
            # Calculate next Fibonacci number
            next_fib = sequence[i-1] + sequence[i-2]
            sequence.append(next_fib)
            
            # Send feedback with current sequence
            feedback_msg.sequence = sequence[:]  # Copy the list
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
            
            # Small delay to simulate work
            time.sleep(0.1)
        
        # Complete the goal successfully
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence[:order + 1] if order >= 0 else []
        self.get_logger().info(f'Goal completed successfully. Final result: {result.sequence}')
        return result


def main(args=None):
    rclpy.init(args=args)
    
    fibonacci_server = FibonacciActionServer()
    
    try:
        rclpy.spin(fibonacci_server)
    except KeyboardInterrupt:
        fibonacci_server.get_logger().info('Shutting down Fibonacci action server')
    finally:
        fibonacci_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
