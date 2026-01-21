package org.firstinspires.ftc.teamcode.blackice.core.geometry;

import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;

import java.util.List;

public class CompositePath {
    private final List<PathGeometry> segments;
    private final double totalLength;

    public CompositePath(List<PathGeometry> segments) {
        if (segments.isEmpty()) {
            throw new IllegalArgumentException("List of segments cannot be empty");
        }
        this.segments = segments;
        this.totalLength = segments.stream().mapToDouble(PathGeometry::length).sum();
    }
    
    public List<PathGeometry> getSegments() {
        return segments;
    }
    
    public PathGeometry getSegment(int index) {
        return segments.get(index);
    }
    
    public int getSegmentCount() {
        return segments.size();
    }

    public PathPoint getEndPathPoint() {
        return segments.get(segments.size() - 1).getEndPathPoint();
    }
    
    public double length() {
        return totalLength;
    }
}
