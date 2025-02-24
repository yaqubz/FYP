from djitellopy import Tello

tello = Tello()

tello.connect()

tello.get_flight_time()
tello.get_temperature()

# For setting WiFi Configs: (TBC)

tello.send_control_command("wifisetchannel 006")
# tello.send_command_with_return("set_wifi_band 5")

tello.send_command_with_return("video?")