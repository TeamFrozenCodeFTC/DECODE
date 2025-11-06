package org.firstinspires.ftc.blackice.core.hardware.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.blackice.util.geometry.Vector;

/**
 *
 * Dev Note: Cannot have Drivetrain&lt;T&gt; behavior due to use of wildcard:
 * Drivetrain&lt;?&gt; d = new MecanumDrive();
 * d.applyPowers(d.getPowers()); // d.applyPowers cannot confirm d.getPowers is same type without
 * having the follower have a generic.
 */
public interface Drivetrain {
    void followVector(Vector robotVector, double turningPower);
    
    /**
     * When at zero power, internally brakes the wheels using regenerative braking and
     * back-EMF.
     * Does not seem to lower the voltage of the battery when braking with this.
     */
    void zeroPowerBrakeMode();
    
    /**
     * Makes the wheels coast when at zero power.
     */
    void zeroPowerFloatMode();
    
    void zeroPower();
}
