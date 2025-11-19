package org.firstinspires.ftc.blackice.core.paths.creation;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.blackice.util.geometry.Vector;
import org.firstinspires.ftc.blackice.core.follower.Follower;

import org.firstinspires.ftc.teamcode.testing.Menu;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group="Black-Ice Examples")
public class PathMaker extends LinearOpMode {
    private final List<Vector> points = new ArrayList<>(); // stores {x, y} points
    private Menu menu;
    
    @Override
    public void runOpMode() {
        Follower follower = new Follower(hardwareMap);

        menu = new Menu(gamepad1);
        
        // Option: add current position
        menu.addOption("Add Current Pos", () -> {
            points.add(follower.getMotionState().position.map(c -> Math.round(c * 100.0) / 100.0));
            menu.unconfirmOption();
        }, () -> {});
        
        // Option: remove last point
        menu.addOption("Remove Last", () -> {
            if (!points.isEmpty()) {
                points.remove(points.size() - 1);
            }
            menu.unconfirmOption();
        }, () -> {});
        
        menu.addOption("Clear All", () -> {
            points.clear();
            menu.unconfirmOption();
            }, () -> {});
        
        waitForStart();
        
        while (opModeIsActive()) {
            follower.update();
            menu.update();

            telemetry.addData("position", follower.getMotionState().position);
            telemetry.addData("heading",
                              Math.round(Math.toDegrees(follower.getMotionState().heading) * 5.0) / 5.0);
            
            // show menu
            telemetry.addLine(menu.getDisplay());
            
            // show all points
            StringBuilder pts = new StringBuilder("Points:\n");
            for (int i = 0; i < points.size(); i++) {
                Vector p = points.get(i);
                pts.append(i).append(": ")
                    .append(p)
                    .append("\n");
            }
            telemetry.addLine(pts.toString());
            
            telemetry.update();
        }
    }
}
