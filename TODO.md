# TODO

- [ ] Implement PID controller to allow the robot to drive in a straight line

- [ ] Readd encoder support for help with turning and dead reckoning

- [ ] Implement a better way of turning (maybe using the encoders / sensors to get a more accurate turn)

- [ ] Look at the mark scheme and see what I can do to get more marks

- [ ] Implement BLE override

- [ ] Implement point cloud mapping via BLE and tkinter

- [ ] add "rave mode" to the robot


## Needed for PID
- [ ] Refactor motor class to allow for easier and more precise control of the motors (allowing for pid to overshoot for faster response)

- [ ] figure out if pid should be over the system as a whole or over each motor individually - i think it should be over the system as a whole as this is what matters not the individual motors

