from djitellopy import Tello
import time

tello = Tello()

tello.connect()

tello.takeoff()

tello.move_down(40)
tello.move_up(30)
tello.move_forward(100)
tello.move_back(100)
tello.rotate_clockwise(360)
time.sleep(2)

tello.land()




