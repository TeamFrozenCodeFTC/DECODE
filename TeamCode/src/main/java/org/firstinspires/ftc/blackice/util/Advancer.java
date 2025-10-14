package org.firstinspires.ftc.blackice.util;

import java.util.ArrayList;
import java.util.List;

/**
 * a utility class that helps in iterating through an array of steps, keeping track
 * of the index, current item, and when it is completed.
 * <p>
 * Used in the PathRoutineController to manage the progression through the paths and
 * actions it is following.
 */
public class Advancer<T> {
    private final ArrayList<T> steps;
    private int index = -1;

    public Advancer(List<T> initialSteps) {
        this.steps = new ArrayList<>(initialSteps);
    }
    
    public T get(int index) {
        return steps.get(index);
    }
    
    public T current() {
        return get(index);
    }
    
    public boolean advance() {
        if (isDone()) return false;
        index++;
        return true;
    }
    
    public boolean isDone() {
        return index >= steps.size() - 1;
    }
    
    public int getIndex() {
        return index;
    }

    public void extendQueue(List<T> moreSteps) {
        steps.addAll(moreSteps);
    }
    
    public int amountOfSteps() {
        return steps.size();
    }
}
