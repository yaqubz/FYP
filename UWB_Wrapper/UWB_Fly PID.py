# 15 Jan Testing - Source: ChatGPT

from UWB_ReadUDP import get_target_position
from djitellopy import tello
import threading
import time

lastpos = None

def update_target_position():
    global lastpos
    while True:
        try:
            lastpos = get_target_position(target_id=1)
        except Exception as e:
            print(f"Error in get_target_position thread: {e}")
        time.sleep(0.1)

class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def compute(self, current_value):
        error = self.setpoint - current_value
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

def main():
    # Start UWB position update thread
    target_thread = threading.Thread(target=update_target_position, daemon=True)
    target_thread.start()

    # Drone initialization
    drone = tello.Tello()
    drone.connect()
    print(f"Battery: {drone.get_battery()}%")

    # Safety check before takeoff
    if drone.get_battery() < 10:
        print("Battery too low. Exiting.")
        return

    # Take off
    drone.takeoff()
    print("Drone took off.")

    # PID Controllers for x and y axes
    x_pid = PIDController(kp=0.5, ki=0.01, kd=0.1, setpoint=2)
    y_pid = PIDController(kp=0.5, ki=0.01, kd=0.1, setpoint=2)

    # Tolerance for landing
    tolerance = 0.05

    try:
        while True:
            if lastpos is not None:
                x_current, y_current, _ = lastpos

                # Calculate control outputs for x and y axes
                x_control = x_pid.compute(x_current)
                y_control = y_pid.compute(y_current)

                # Clip control outputs to fit the drone's speed limits
                x_control = max(min(x_control, 20), -20)
                y_control = max(min(y_control, 20), -20)

                # Command the drone
                drone.send_rc_control(int(x_control), int(y_control), 0, 0)

                # Check if the drone is within the tolerance
                distance_to_target = ((x_current - 2) ** 2 + (y_current - 2) ** 2) ** 0.5
                print(f"Current Position: ({x_current:.2f}, {y_current:.2f}), Distance to Target: {distance_to_target:.2f}")

                if distance_to_target <= tolerance:
                    print("Target reached. Landing...")
                    drone.land()
                    break

                time.sleep(0.1)

    except KeyboardInterrupt:
        print("Interrupted by user. Landing...")
        drone.land()
    finally:
        drone.end()
        print("Drone flight ended.")

if __name__ == "__main__":
    main()
