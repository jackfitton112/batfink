# BATFINK AUTONOMOUS ROBOT

![BATFINK](imgs/image.png)

(Image: BATFINK) 

## Introduction

This Project was developed for my third year RDaC (Robotic Design and Construction) module at the University of York. I is based off the Arduino Nano BLE 33 and the ARB Board.

The project is a robot that can autonomously navigate a maze and find a target. The robot is to be controlled by RTOS (Real Time Operating System) and is to be programmed in C++ using Mbed libraries.

I have named the robot BATFINK, which stands for **B**atfink **A**utonomous **T**arget **F**inding **I**ntelligent **N**avigation **K**inetic bot also as it uses ultrasonic sensors to navigate and find a target just as the 90s cartoon character Batfink would (SUPER SONIC SONAR RADAR).

## Hardware

TBD - Unfortunatly, I am running out of time to do the project therefore this doc will be neglected for a while.

## Software

TBD - see above






- TODO:

PID:

The output speed has been lowered from +/- 1 to +/- 0.5, this has shown great results with trying to stop oscillations, it is also much more accurate.  (DONE)

31/12/23 01:25
Update: To try and resolve the angle issues, PID should be tuned more. Also reduce the error threshold on the move thread in robot.cpp to try and get the robot to stop closer to the target. currently its +/- 3% which over 180 degrees (approx 200mm of travel) is 6mm, this is too much. 1% would be ideal. (DONE) - Slowed the robot down to 0.3 and its alot more accurate but now it has issues driving on carpet.

31/12/23 02:39
instead of having this clutter up a source file ill put it here:

 * I Think this needs rewriting, its not working properly
 * when the motor is told to turn left it goes forward at first before turning which is wrong
 * it should just turn on the spot not go forward
 * I think its to do with the way that pid is enabled
 * When you tell it to go to a position it goes and then the PID panics and 
 * tries to correct it but it cant because the motor is already at the target position
 * but then undefined behaviour happens and it goes forward or spins trying to catch up
 * 
 * TODO: Rewrite PID control or mess with the constants to try and improve it
 * update: Problem has been looked into and now i think the PID is working alot better, were down to 5% error
 * will try lowering the error value to 1% and see what happens
 * 
 * 1% is far too small for this robot, at +/-3% it oscillates around the far too quickly to actually stop.
 * some damping via more turning would be good
 * 
 * Another option that has shown good results is to add some more weight to the robot,
 * This increases the friction, reduces slip and makes the wind up less of a problem
 * 
 * This function is called by the PID ticker. It calculates the PID output for the motor
 * based on the current velocity and the target velocity.
 * 
 * setting the max speed to 0.8 has stopped the oscillations completely, to try and give some more accuracy, im going to
 * drop this to 0.6 and see what happens
 * 

Robot:

Currently the turn to angle calculation is overshooting, this is by a considerable amount, around 10% (180 is giving approx 200 degrees).
the proposed fix is that im going to add an offset to the turn angle function to try and compensate for this. (DONE)

31/12/23 01:25
Update: Still having angle issues, its still overshooting on carpet and undershooting when the motors are off the ground, this would say a PID error which may be possible, more tuning is required. (DONE)

31/12/23 01:37
Update: Think this is resolved but with how this robot has been going it would be hard to say

31/12/23 01:38
TODO next: now that the robot seems to be behaving the next step is to get the robot to go to coordinates, i want it to drive to where it thinks the coords are and then check if its there (within 50mm is fine), ideally this would use sensor fusion to make sure the robot is actually at the coords and not just the right distance away.

31/12/23 01:55
its all fucked, im going to bed if i cant get it working in the next 20 mins

31/12/23 02:22
This ive figured it out, the move to coords func used pythag to calc distance but this didnt take into account dx or dy being negitive, will try this and then hopefully everything will be working abit better!

31/12/23 02:32
The issue seems to have been that the motor function was returning as the thread delt with everything so it was calculating the difference immediately rather letting the motors run. this has been sorted, still having some minor angle issues with the robot, told it to go (0,0) -> (300,0) -> (0,100) and it ended up at (41,11) This means there is probs a cock up with the angle calc but my head hurts and i truely cant be fucked with this shit anymore! will have a think about it in the morning but tomorrow isnt being ruined by uni work, its NYE im going clubbing.

31/12/23 02:53
It works!! the issue was i was passing rads to a func that takes deg, it seems to function well now, (0,100) leads to (0,88) which is within spec. - after  a few mins of testing (03:00) i have found there a still some issues, nothing major but it has a panic now ive reduced the tolerance to 15mm and and makes strange choices about the angles. Think i will leave it here for tonight as i cant be fucked :).

TODO tomorrow:
-finish gotoXY sys, should be easy with a fresh brain
-work on mapping, i would like the map to be dynamic but that means using vectors which are scary cos memory issues
-intergrate gotoXY and mapping so the maze can be solved.
- write a crused "explore" algorithm so it tries places it hasnt been to over ones it has. 
- intergrate sensors back in for obsticle avoidance. 

Thats probs 2-3 days of works, I need to start control revision as of the 2/1/24 in order to have any chance so the robot might get put off until after control has been done!!

right... its 03:03, im done!

02/01/24 18:52:
After having a few days off i am back on robot development, I think my coords system is working quite well but it is very hard to test it as i dont have much floor space, I think the next big thing i am gonig to do is itergrate BLE back into the robot, not only will this help with debug but i can use it to send control signals from a laptop which will save me a shitload of time.

I want BLE to do the following:
- publish robot X, Y, Theta values
- publish robot speed
- publish sensor values (ultrasonic) for point cloud mapping
- recieve control signals from a laptop (goto X,Y, stop etc)

This will all be done in comms.cpp / .h and will be done using the BLE library.

16/1/24 13:28:

Control exam is now out of the way and its back to robot, the whole project needs overhauling and i need to get an MVP working first with all of the required features for the marks then i can make it fancy and add bits but weve got 4 days to get this finished (8 if i self cert) and i want a first for this module so im going to have to work hard.

what the robot needs to do:
- Move forward, left and right accurately - This means i need to spend longer tuning my PID but this should be easier now i know how control works.
- record its position and orientation
- store this data along with the sensor data to map the maze
- use this data to solve the maze and come back in the most efficient way possible

This is going to be alot of work but this plan needs sticking to.


