package org.firstinspires.ftc.blackice.tuning;

import com.acmerobotics.dashboard.config.Config;


/**
 * Tune the robot's forward/backward braking distances.
 * <p>
 * Sets the wheel's motors to zero power brake mode, runs the robot at different velocities,
 * sets the power to zero (braking), and calculates the braking distance.
 * The robot runs two tiles forward back and forth.
 *
 * <p>
 * After running, plug the given constants into {@link TuningConstants#FORWARD_BRAKING_DISPLACEMENT}
 */
@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "Tuning")
public class ForwardBrakingTuner extends DistanceTuner {
    @Override
    public void runOpMode() {
        run(0);
    }
}
