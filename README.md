## Road Runner

A simple Kotlin library for planning 2D mobile robot paths and trajectories designed for FTC. The original Road-Runner repository can be found at [this link](https://github.com/acmerobotics/road-runner)

## Raiku Runner

A modified version of Road Runner v0.5.5 that implements a custom Bézier curve spline and modified interpolator. It also includes the v1.0 feature of asymetric accel and decel.

The current documentation on my changes is quite scarce but will be improved when i get a bit of free time, also the asymetric accel can be set only for the funnyRaikuSpline.

# An example
An example trajectory would be:
```
TrajectorySequence traj = drive.trajectorySequenceBuilder()
                               .funnyRaikuCurve(P1, R1, R2, H1, H2, vc, ac, dc)
                               .build();
```

where P1 is a Pose2d with the end pose, R1 and R2 are the control points for the bezier curve, H1 and H2 are control values for the interpolator and vc, ac, dc are VelocityConstraint and AccelConstraints.

R1 and R2 are in polar coordinates with reference to the start and end pose, with R1.x being the length and R1.y being the angle with respect to OX.

H1 and H2 are control values for the formula $3x(1-x)^3a + 3x(1-x)b + x^3$ which defines the heading during the trajectory. This was useful as i could not get the stock interpolators to get the robot to the correct heading before reaching the stack, usually resulting in it toppling the stack.

A useful tool for finding good values is just trying them out in [desmos](https://www.desmos.com/calculator/9nigdxolvm) where 0 is the start heading and 1 is the end heading. Some good values for $a$ and $b$ are: (0.5, 1) for a curve that goes to the final heading quickly and then stays there, and (0, 0.5) for a near linear interpolator. 

Vc and ac are part of the stock road runner and define a custom max velocity and acceleration for the current trajectory while dc defines the max deceleration for it.

# Visualisation

A good way of visualising the resulting curves for rapid prototyping without any external dependencies is just using the built in trajectory visualiser.

I personally use a modified version of the function ``draw()`` found in the roadrunner-quickstart, and just look at the resulting curve in the [ftc-dashboard](https://github.com/acmerobotics/ftc-dashboard). The slightly modified ``draw`` function being
```
private void draw(Canvas fieldOverlay, TrajectorySequence sequence) {
    if (sequence != null) {
        for (int i = 0; i < sequence.size(); i++) {
            SequenceSegment segment = sequence.get(i);

            if (segment instanceof TrajectorySegment) {
                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke(COLOR_INACTIVE_TRAJECTORY);

                DashboardUtil.drawSampledPath(fieldOverlay, ((TrajectorySegment) segment).getTrajectory().getPath());
            } else if (segment instanceof TurnSegment) {
                Pose2d pose = segment.getStartPose();

                fieldOverlay.setFill(COLOR_INACTIVE_TURN);
                fieldOverlay.fillCircle(pose.getX(), pose.getY(), 2);
            } else if (segment instanceof WaitSegment) {
                Pose2d pose = segment.getStartPose();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke(COLOR_INACTIVE_WAIT);
                fieldOverlay.strokeCircle(pose.getX(), pose.getY(), 3);
            }
        }
    }
}
```
Which is then called like so.
```
TelemetryPacket packet = new TelemetryPacket();
draw(packet.fieldOverlay(), traj);
dashboard.sendTelemetryPacket(packet);
```

## Installation

The compiled jar file can be found at ``core/build/libs/core.jar`` which can then be copied into the quickstart in the directory ``TeamCode/lib``. After this you can comment out the line that told the project to use the stock road runner, which probably looked like this ``implementation 'com.acmerobotics.roadrunner:core:0.5.6'``, and add the line ``implementation files('lib/core.jar')`` to tell it to use the custom road-runner jar.

Those using ``TrajectorySequence``s inside the quickstart will also need to add the following functions to the ``trajectorysequence/TrajectorySequenceBuilder.java`` file relative to where the code for the robot usually goes.

```
public TrajectorySequenceBuilder funnyRaikuCurve(Pose2d endPosition, Vector2d p1, Vector2d p2, double h1, double h2) {
    return addPath(() -> currentTrajectoryBuilder.funnyRaikuCurve(endPosition, p1, p2, h1, h2, currentVelConstraint, currentAccelConstraint, currentAccelConstraint));
}

public TrajectorySequenceBuilder funnyRaikuCurve(Pose2d endPosition,
                                                 Vector2d p1,
                                                 Vector2d p2,
                                                 double h1,
                                                 double h2,
                                                 TrajectoryVelocityConstraint velConstraint,
                                                 TrajectoryAccelerationConstraint accelConstraint,
                                                 TrajectoryAccelerationConstraint decelConstraint
) {
    return addPath(() -> currentTrajectoryBuilder.funnyRaikuCurve(endPosition, p1, p2, h1, h2, velConstraint, accelConstraint, decelConstraint));
}
```

The resulting visualisation should look like something along the lines of
<p align="center">
    <img src="https://media.discordapp.net/attachments/944145687896002640/1073313202311544913/Teren.png"/>
</p>
## Notes

I will probably add asymetric acceleration and deceleration to all of the available trajectories once i get the time to. 

For those using OnBotJava, i'm sorry. I've no idea how to get it to work there...

Happy coding!
