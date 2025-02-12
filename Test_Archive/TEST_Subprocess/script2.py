from djitellopy import tello
import cv2

def main():
    drone = tello.Tello()
    drone.connect()
    print(drone.get_battery())
    drone.takeoff()
    drone.move_up(20)
    drone.end()

if __name__ == '__main__':
    main()