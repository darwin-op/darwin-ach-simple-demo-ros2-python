# Darwin-Ach Simple Demo for Ros2 + Python 3

This is a quick demonstration about the utilisation of the Darwin-Ach system to control the Darwin-OP using Ros2 and Python3 via the Ros2Ach bridge

## Step 0: Notes about setup

If you setup the Darwin Lofaro Legacy system to auto start on the Darwin-OP and on the backpack comptuer then:

* Enable the robot by pressing the start button in the "full body reference" mode.  The robot will become stiff.
* Now you can skip to Step 3.

## Step 1: Start the Darwin-OP

Follow the normal process to turn on the Darwin-OP and to turn it on.  Please reference the Darwin-Lofaro-Legacy / Darwin-Ach manual for startup prosedures.  For your reference we will be running the Darwin-Ach server on the Darwin-OP.

1. Ensure the Darwin-OP is suspended in the air by the strap on its back.

2. Login to the Darwin-OP via SSH

3. Start the Darwin-Ach Server on the Darwin-OP.

```
$ darwin-ach darwin start
```

4. Turn on the Darwin-OPs actuators.
```
$ darwin-ach power on
```

## Step 2: Start the client on a the backpack computer

On the backpack computer you will need run the Darwin-Ach client and the Ros2Ach bridge that computer.

1. Login to the Darwin-OPs backpack computer 

2. Start the Darwin-Ach Client and the Ros2Ach bridge on the backpack computer
```
$ darwin-ach darwin-ros start
```

## Step 3: Run the example.

This example will set the angle of the joints 19, 6, 5, 1, and 2 to 20.0, 20.0, 20.0, 20.0, and -20.0 def.  The example then sleeps for 3.0 seconds.  The system then sets the the same joints to -0.35, -0.35, -0.35, -0.35, and 0.35 rad respectively. The system will then sleep for another 3.0 seconds.  The loop will the restart and run until it is stopped. 

Note: The message type of "String" is chosen so you do not need any special or custum messages are required.  This will allow better future compatiability. 

1. Enter the example directory
```
$ cd darwin-ach-simple-demo-ros2-python
```

This will make an executable named "test"

2. Run the executable
```
$ python3 test.py
```

## Code Explained

1. Import the primary Ros2 and Python headers
```
from time import sleep
import time
import rclpy
```

2. Import the message type required for the Ros2 topics
```
from std_msgs.msg import String
```

3. Initilize Ros2
```
rclpy.init(args=args)
```

4. Set Node name
```
node = rclpy.create_node('darwin_simple_demo_ros2_python_publisher')
```

5. Publisher for the staged reference positions 
```
publisher = node.create_publisher(String, '/darwin/ref/position', 10)
```

6. Publisher for posting the staged reference positions to the motors
```
publisher_cmd = node.create_publisher(String, '/darwin/cmd', 10)
```

7. String message type
```
msg = String()
```



8. Run loop while Ros2 is active
```
while rclpy.ok():
```

9. Stage motors 19, 6, 5, 1, and 2 with units of deg
```
msg.data = 'd 19 20.0 6 20.0 5 20.0 1 20.0 2 -20.0'
publisher.publish(msg)
```

10. Send the staged values to the motors via the ros2ach bridge
```
msg.data = 'post'
publisher_cmd.publish(msg)
```

11. Sleep for 3 seconds
```
sleep(3.0)
```

12. Stage motors 19, 6, 5, 1, and 2 with units of rad
```
msg.data = 'r 19 -0.35 6 -0.35 5 -0.35 1 -0.35 2 0.35'
publisher.publish(msg)
```

13. Send the staged values to the motors via the ros2ach bridge
```
msg.data = 'post'
publisher_cmd.publish(msg)
```

14. Sleep for 3 seconds
```
sleep(3.0)
```

15. Kill Node
```
node.destroy_node()
rclpy.shutdown()
```


# Darwin-Ach Simple Demo for Ros2 + Python 3 Using the darwin_ach.py class

This is a quick demonstration about the utilisation of the Darwin-Ach system to control the Darwin-OP using Ros2 and Python3 via the Ros2Ach bridge while using the darwin_ach.py class

## Step 0: Notes about setup

If you setup the Darwin Lofaro Legacy system to auto start on the Darwin-OP and on the backpack comptuer then:

* Enable the robot by pressing the start button in the "full body reference" mode.  The robot will become stiff.
* Now you can skip to Step 3.

Else:

Follow the startup steps as defined in the previous example.

## Step 1: Run the example

This example will set the angle of the joints 19, 6, 5, 1, and 2 to 20.0, 20.0, 20.0, 20.0, and -20.0 def.  The example then sleeps for 3.0 seconds.  The system then sets the the same joints to -0.35, -0.35, -0.35, -0.35, and 0.35 rad respectively. The system will then sleep for another 3.0 seconds.  The loop will the restart and run until it is stopped. 

Note: The darwin_ach.py class takes care of the message type and conversion.

To run the example run the following:
```
$ python3 test_class_ref.py
```

## Code Explained

1. Import the headers
```
from time import sleep
import darwin_ach as da
```

2. Make Darwin Ach Ros Object
```
dar = da.DarwinAchRos()
```

3. Stage and set motors 19, 6, 5, 1, and 2 with units of deg
```
mot = (  19,    6,    5,    1,     2)
pos = (20.0, 20.0, 20.0, 20.0, -20.0)
dar.setMotDeg(mot,pos)
```

4. Sleep for 3 seconds
```
sleep(3.0)
```

5. Stage and set motors 19, 6, 5, 1, and 2 with units of rad
```
mot = (   19,     6,     5,     1,    2)
pos = (-0.35, -0.35, -0.35, -0.35, 0.35)
dar.setMot(mot,pos)
```

6. Sleep for 3 seconds
```
sleep(3.0)
```

7. Kill Node
```
dar.close()
```

# Darwin-Ach Simple Demo for Ros2 + Python 3 Using the darwin_ach.py class and state feedback

This is a quick demonstration about the utilisation of the Darwin-Ach system to read the state values of the Darwin-OP using Ros2 and Python3 via the Ros2Ach bridge while using the darwin_ach.py class

## Step 0: Notes about setup

If you setup the Darwin Lofaro Legacy system to auto start on the Darwin-OP and on the backpack comptuer then:

* Enable the robot by pressing the start button in the "full body reference" mode.  The robot will become stiff.
* Now you can skip to Step 3.

Else:

Follow the startup steps as defined in the previous example.

## Step 1: Run the example

This example will read and print the state values of the Darwin using the darwin_ach.py class.  The IPC used is Ros2 thus this can be run on any computer on the same Ros2 network as the robot.  By default only the IMU feeds back data.  This example turns on full state feedback (i.e. right and left FT sensors and all joint state data).  Because of this the control rate is slower.

To run the example run the following:
```
$ python3 test_class_state.py
```

Note: You can use ros to read the topics in your own custom software, however this is an example using the darwin_ach.py class.

## Code Explained 

1. Import headers
```
from time import sleep
import darwin_ach as da
```

2. Print IMU Acc Values
Each of the values are a float. Units: m/s^2
```
print("IMU: Acc = ",end='')
print(dar.imu_acc_x, end='')
print(' ', end='')
print(dar.imu_acc_y, end='')
print(' ', end='')
print(dar.imu_acc_z)
```

2. Print IMU Gryo Values
Each of the values are a float. Units: rad/s
```
print("IMU: Gyro = ",end='')
print(dar.imu_gyro_x, end='')
print(' ', end='')
print(dar.imu_gyro_y, end='')
print(' ', end='')
print(dar.imu_gyro_z)
```

3. Print FT Left
Each of the values are a float.  Units for x and y values are in N.  Units for lift state is a boolean.
```
print("FT Left = ", end='')
print(dar.ft_left_x, end='')
print(' ', end='')
print(dar.ft_left_y, end='')
print(' ', end='')
print(dar.ft_left_lift, end='')
print(' ', end='')
print(dar.ft_left_lift_x, end='')
print(' ', end='')
print(dar.ft_left_lift_y)
```

4. Print FT Right
Each of the values are a float.  Units for x and y values are in N.  Units for lift state is a boolean.
```
print("FT Right = ", end='')
print(dar.ft_right_x, end='')
print(' ', end='')
print(dar.ft_right_y, end='')
print(' ', end='')
print(dar.ft_right_lift, end='')
print(' ', end='')
print(dar.ft_right_lift_x, end='')
print(' ', end='')
print(dar.ft_right_lift_y)
```

5. Print Motor Position
This is an array of length number of motors + 1.  Each value in the array is the given state value and is denoted via a float.  Units: rad
```
print("Motor Position: ",end='')
print(dar.motor_position)
```

6. Print Motor Velocity
This is an array of length number of motors + 1.  Each value in the array is the given state value and is denoted via a float.  Units: rad/sec
```
print("Motor Velocity: ",end='')
print(dar.motor_velocity)
```

7. Print Motor Torque
This is an array of length number of motors + 1.  Each value in the array is the given state value and is denoted via a float. Units: percent (0.0 to 1.0)
```
print("Motor Torque: ",end='')
print(dar.motor_torque)
```

8. Print Motor Voltage
This is an array of length number of motors + 1.  Each value in the array is the given state value and is denoted via a float. Units: volts
```
print("Motor Voltage: ",end='')
print(dar.motor_voltage)
```

9. Print Motor Temperature
This is an array of length number of motors + 1.  Each value in the array is the given state value and is denoted via a float. Units: Celsius
```
print("Motor Temperature: ",end='')
print(dar.motor_temperature)
```

10. Sleep for 0.01 sec
```
      sleep(0.01)
```

11. Kill Node
```
    dar.close()
```




