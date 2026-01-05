package org.firstinspires.ftc.teamcode.auto.steps;

import java.util.ArrayList;
import java.util.List;

public class StepRunner {
    private final List<Step> steps = new ArrayList<>();
    private int index = 0;

    public void add(Step step) {
        steps.add(step);
    }

    public void run() {
        if (index >= steps.size()) return;

        Step step = steps.get(index);
        step.run();

        if (step.isFinished()) {
            index++;
        }
    }
}
