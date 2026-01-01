from djitellopy import Tello
import time

tello = Tello()

tello.connect()

tello.streamon()


# wait user push q don@t get v'deo I w'll hande
while True:
    if( input("Press q to quit: ") == 'q'):
        break

tello.streamoff()


tello.end()





