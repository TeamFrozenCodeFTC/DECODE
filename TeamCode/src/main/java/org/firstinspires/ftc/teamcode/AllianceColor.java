package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.blackice.util.geometry.Vector;

public enum AllianceColor {
    RED(new Vector(144-6, 144-6), new Vector(48-9, 24+9)),
    BLUE(new Vector(6, 144-6), new Vector(24*4+9, 24+9));
    
    // start pos rotate 180
    
    private final Vector goalPosition;
    private final Vector basePosition;
    
    AllianceColor(Vector goalPosition, Vector basePosition) {
        this.goalPosition = goalPosition;
        this.basePosition = basePosition;
    }

    public Vector getGoalPosition() {
        return goalPosition;
    }
    
    public Vector getBasePosition() {
        return basePosition;
    }
}
