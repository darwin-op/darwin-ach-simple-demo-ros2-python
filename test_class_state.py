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
import darwin_ach as da

def main(args=None):
    # Make Darwin Ach Ros Object
    dar = da.DarwinAchRos(state=True)

    while True:
      # Print IMU Acc Values
      print("IMU: Acc = ",end='')
      print(dar.imu_acc_x, end='') 
      print(' ', end='')
      print(dar.imu_acc_y, end='') 
      print(' ', end='')
      print(dar.imu_acc_z) 

      # Print IMU Gryo Values
      print("IMU: Gyro = ",end='')
      print(dar.imu_gyro_x, end='') 
      print(' ', end='')
      print(dar.imu_gyro_y, end='') 
      print(' ', end='')
      print(dar.imu_gyro_z) 

      # Print FT Left
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

      # Print FT Right
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

      # Print Motor Position
      print("Motor Position: ",end='')
      print(dar.motor_position)

      # Print Motor Velocity
      print("Motor Velocity: ",end='')
      print(dar.motor_velocity)

      # Print Motor Torque
      print("Motor Torque: ",end='')
      print(dar.motor_torque)

      # Print Motor Voltage
      print("Motor Voltage: ",end='')
      print(dar.motor_voltage)

      # Print Motor Temperature
      print("Motor Temperature: ",end='')
      print(dar.motor_temperature)

      sleep(0.01)

    # Kill Node
    dar.close()


if __name__ == '__main__':
    main()

