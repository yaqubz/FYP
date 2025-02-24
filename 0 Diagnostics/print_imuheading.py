"""4 Feb for printing IMU heading
IMPT CONCLUSION: Tello's YAW = 0 is inititalized when it is first turned on. 
[-180, 180] where CCW is negative i.e. drone turns left
"""


from djitellopy import Tello
import time

tello = Tello()
tello.connect()
tello.get_battery()


while True:
    print(tello.get_yaw())
    # print(tello.get_distance_tof())
    # print(tello.get_height())   # somehow this changes when the Tello is moving up/down, but not when stationary?
    time.sleep(0.2)

