package org.firstinspires.ftc.teamcode.blackice.core.commands;

import org.firstinspires.ftc.teamcode.blackice.core.FollowPathCommand;
import org.firstinspires.ftc.teamcode.blackice.core.HeadingInterpolator;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.miniblackice.core.geometry.LineGeometry;

import java.util.ArrayList;
import java.util.List;

public class AutoRoutine {
    public List<Command> routineSteps = new ArrayList<>();

    private int index = 0;

    Pose previousPose;
    
    public AutoRoutine(Pose startingPose) {
        this.previousPose = startingPose;
    }
    
    public Pose getEndPose() {
        for (int i = routineSteps.size() - 1; i >= 0; i--) {
            Command command = routineSteps.get(i);
            if (command instanceof FollowPathCommand) {
                FollowPathCommand pathCommand = (FollowPathCommand) command;
                return pathCommand.endPose;
            }
        }
        return null;
    }

    public void run() {
        if (index >= routineSteps.size()) return;

        Command command = routineSteps.get(index);
        command.update();

        if (command.isFinished()) {
            index++;
            if (index >= routineSteps.size()) return;
            routineSteps.get(index).start();
        }
    }
    
    public void start() {
        routineSteps.get(index).start();
    }

    public void add(Command step) {
        routineSteps.add(step);
    }

    public void addAction(Runnable action) {
        routineSteps.add(Command.singleAction(action));
    }
    
    public void addRoutine(AutoRoutine autoRoutine) {
        routineSteps.addAll(autoRoutine.routineSteps);
    }
    
    public int getIndex() {
        return index;
    }
}
