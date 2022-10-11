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
    dar = da.DarwinAchRos()

    while True: 
      # Stage motors 19, 6, 5, 1, and 2 with units of deg
      mot = (  19,    6,    5,    1,     2)
      pos = (20.0, 20.0, 20.0, 20.0, -20.0)
      dar.setMotDeg(mot,pos)
      print(pos)

      # Sleep for 3 seconds
      sleep(3.0)

      # Stage motors 19, 6, 5, 1, and 2 with units of rad
      mot = (   19,     6,     5,     1,    2)
      pos = (-0.35, -0.35, -0.35, -0.35, 0.35)
      dar.setMot(mot,pos)
      print(pos)

      # Sleep for 3 seconds
      sleep(3.0)

    # Kill Node
    dar.close()


if __name__ == '__main__':
    main()

