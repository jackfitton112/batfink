# Todo With mapping

> I am now very far behind with RDaC, for where I want to be

My robot is very cool and does lots of things but non of them are assessed. From now on I need to get my robot to a point where it can be assessed.

before mapping can begin there are a few things that need to be done:
- [ ] Get ble working - This is going to be required to send sensor data to the robot - this can all be sent as one characteristic. 
- [ ] Get threading working more efficiently, currently the usonic sensors are being quite the pain in the arse, there is alot of technical debt here that needs to be sorted out.
- [ ] Make sure that the encoders are working correctly, this is going to be required for both mapping and PID.


Ideally there would be some functions where the robot does the following:
- [ ] Robot measures distance to wall and then drives forward until it is 5cm away from the wall. this negates the need to keep checking the front sensor and will speed up the control loop.
- [ ] Robot turn to a specific angle, this will be very useful when mapping

for mapping we also want the robot to take a shit load of measurements, such as at each 10cm it spins round and measures the distances around it so a point cloud can be drawn up 