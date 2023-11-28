# Reafctoring plan

- [X] Rewrite motor control lib to be more thread friendly, make sure its non blocking and works as planned
- [ ] Rewrite BLE so that simple flags can be set (stop / start, mode etc).
- [X] Rewrite Sensor reading lib to be more thread friendly, make sure its non blocking and works as planned
- [ ] Write PID controller
- [ ] Look into mapping algorithms - how can i store the map on the robot? How can i make it more efficient?
- [ ] Look into path planning algorithms - how can i make it more efficient?

This all needs to be done on thursday, its going to be a very busy day.


the maze is 2m x 1.5m the obsticles are 8 x 26cm

a good amount of cells for the map would be 20 x 15, this should be stored in a 2d array of 20 x 15, when a 