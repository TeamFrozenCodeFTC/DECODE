package org.firstinspires.ftc.blackice.tuning;

import com.acmerobotics.dashboard.config.Config;


@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "Tuning")
public class LateralBrakingTuner extends DistanceTuner {
    @Override
    public void runOpMode() {
        run(90);
    }
}
