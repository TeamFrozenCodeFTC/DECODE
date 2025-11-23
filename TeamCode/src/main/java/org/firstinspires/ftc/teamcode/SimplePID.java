package org.firstinspires.ftc.teamcode;

import java.util.LinkedList;

class Observation {
    public double error;
    public double time;
    public double deltaTime;

    public Observation(double error, double time, double deltaTime) {
        this.error = error;
        this.time = time;
        this.deltaTime = deltaTime;
    }
}



public class SimplePID {
    static final int MAX_OBSERVATIONS = 5;
    double Kf = 0.0;
    double Kp = 0.0;
    double Kd = 0.0;
    double Ki = 0.0;
    double target = 0.0;
    LinkedList<Observation> observations = new LinkedList<>();

    private Observation addObeservation(double actual) {
        double error = actual - target;
        Observation lastObservation = observations.size() > 0 ? observations.getFirst() : null;
        double currentTime = System.currentTimeMillis();
        Observation currentObservation = new Observation(
                error,
                currentTime,
                lastObservation == null ? 0.0 : currentTime - lastObservation.time);

        observations.add(currentObservation);
        if(observations.size() > 5) {
            observations.removeLast();
        }

        return currentObservation;
    }

    public SimplePID(double kf, double kp, double kd, double ki) {
        Kf = kf;
        Kp = kp;
        Kd = kd;
        Ki = ki;
    }

    public void setTarget(double target) {
        this.target = target;
        observations.clear();
    }

    public double getOutput(double actual) {
        Observation obs = addObeservation(actual);
        

    }
}
