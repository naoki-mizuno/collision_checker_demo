# collision_checker_demo

collision_checker_demo package.

## How to use

```
roslaunch collision_checker_demo demo.launch
# Use the 2D Pose Estimate tool in Rviz to specify robot pose
# If pose is valid, robot polygon will be shown in green, otherwise in red
```

## Caveats

The collision check result may not coincide with what's shown for the robot
footprint. This is due to the fact that the robot's pose is not rounded to the
nearest grid resolution. By playing around with this demo you can get an idea
of how much the rounding error is compared to the size of your robot.

## License

MIT


## Author

Naoki Mizuno (mizuno.naoki@rm.is.tohoku.ac.jp)
