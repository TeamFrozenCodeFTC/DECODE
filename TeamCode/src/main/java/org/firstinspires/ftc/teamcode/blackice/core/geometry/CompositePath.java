//package org.firstinspires.ftc.teamcode.miniblackice.core.geometry;
//
//import org.firstinspires.ftc.teamcode.miniblackice.geometry.Pose;
//import org.firstinspires.ftc.teamcode.miniblackice.geometry.Vector;
//
//import java.util.List;
//
//public class CompositePath implements PathGeometry {
//    private final List<PathGeometry> segments;
//    private final double totalLength;
//
//    public CompositePath(List<PathGeometry> segments) {
//        this.segments = segments;
//        this.totalLength = segments.stream().mapToDouble(PathGeometry::length).sum();
//    }
//
//    @Override
//    public Pose getClosestPointTo(Vector s) {
//        Ve remaining = s;
//
//        for (PathGeometry segment : segments) {
//            if (remaining <= segment.length()) {
//                return segment.computeClosestPathPointTo(remaining);
//            }
//            remaining -= segment.length();
//        }
//
//        return segments.get(segments.size() - 1)
//                .computeClosestPathPointTo(segments.get(segments.size() - 1).length());
//    }
//
//    @Override
//    public double length() {
//        return totalLength;
//    }
//}
