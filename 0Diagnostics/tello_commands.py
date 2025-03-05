from djitellopy import Tello

tello = Tello()

tello.connect()

# tello.get_flight_time()
# tello.get_temperature()
print(tello.get_battery())

# For setting WiFi Configs: (TBC)

tello.send_control_command("wifisetchannel 006")

# print(tello.get_yaw())
# tello.takeoff()
# print(tello.get_yaw())
# tello.go_xyz_speed(300,0,0,20)      # x: forward +ve | y: left +ve |
# tello.move_forward(300)      # x: forward +ve | y: left +ve |
# tello.land()
# print(tello.get_yaw())