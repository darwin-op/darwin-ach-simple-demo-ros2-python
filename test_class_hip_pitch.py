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
import time

def main(args=None):
    # Make Darwin Ach Ros Object
    dar = da.DarwinAchRos()

    tick = time.time()
    tock = time.time()
    
    # desired setpoint in deg
    des = 10.0    
    ref = 0.0
    L = 30.0
    while True: 
      tock = time.time()
      dt = tock - tick

      # Low Pass Filter
      ref = (ref * (L-1.0) + des) / L
      
      # Change desired reference every 3 seconds
      if dt > 3.0:
        des = -des
        tick = tock

      # Set the position
      mot = (  dar.RHP )
      pos = (  ref    )
      dar.setMotDeg(mot,pos)
      print(pos)

      # Sleep for 0.01 sec
      sleep(0.01)

    # Kill Node
    dar.close()


if __name__ == '__main__':
    main()

