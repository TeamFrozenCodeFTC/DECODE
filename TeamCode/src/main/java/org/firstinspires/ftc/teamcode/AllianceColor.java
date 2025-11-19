package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.blackice.util.geometry.Vector;

public enum AllianceColor {
    // 16.5 is average of robot lengths (16x17)
    BLUE(new Vector(6, 144-6), new Vector(24*4 + 16.5/2, 24 + 16.5/2)),
    RED(BLUE.goalPosition.mirroredAcrossYAxis(), BLUE.basePosition.mirroredAcrossYAxis());
    
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
