package org.firstinspires.ftc.teamcode.blackice.drivetrain;

import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;

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
