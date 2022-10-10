#/*******************************************************************************
#* Copyright 2022 Daniel M. Lofaro
#*
#* Licensed under the Apache License, Version 2.0 (the "License");
#* you may not use this file except in compliance with the License.
#* You may obtain a copy of the License at
#*
#*     http://www.apache.org/licenses/LICENSE-2.0
#*
#* Unless required by applicable law or agreed to in writing, software
#* distributed under the License is distributed on an "AS IS" BASIS,
#* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#* See the License for the specific language governing permissions and
#* limitations under the License.
#*******************************************************************************/
#
#/* Author: Daniel M. Lofaro */

from time import sleep
import time
import rclpy

from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)

    # Set Node name
    node = rclpy.create_node('darwin_simple_demo_ros2_python_publisher')
 
    # Publisher for the staged reference positions 
    publisher     = node.create_publisher(String, '/darwin/ref/position', 10)

    # Publisher for posting the staged reference positions to the motors
    publisher_cmd = node.create_publisher(String, '/darwin/cmd', 10)

    # String message type
    msg = String()

    while rclpy.ok(): 
      # Stage motors 19, 6, 5, 1, and 2 with units of deg
      msg.data = 'd 19 20.0 6 20.0 5 20.0 1 20.0 2 -20.0'
      publisher.publish(msg)
      print(msg.data)

      # Send the staged values to the motors via the ros2ach bridge
      msg.data = 'post'
      publisher_cmd.publish(msg)

      # Sleep for 3 seconds
      sleep(3.0)

      # Stage motors 19, 6, 5, 1, and 2 with units of rad
      msg.data = 'r 19 -0.35 6 -0.35 5 -0.35 1 -0.35 2 0.35'
      publisher.publish(msg)
      print(msg.data)

      # Send the staged values to the motors via the ros2ach bridge
      msg.data = 'post'
      publisher_cmd.publish(msg)

      # Sleep for 3 seconds
      sleep(3.0)


    # Kill Node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

