package org.firstinspires.ftc.teamcode.blackice.geometry;

public class PoseConstructor {
    Vector robotSize;
    
    public PoseConstructor(Vector robotSize) {
        this.robotSize = robotSize;
    }
    
    public Vector bottomRightCorner(Vector vector) { // TODO change based on heading
        return vector.plus(robotSize.dividedBy(2));
    }
}
