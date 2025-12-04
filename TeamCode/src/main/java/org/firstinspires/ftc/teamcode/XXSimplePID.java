package org.firstinspires.ftc.teamcode;

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

public class XXSimplePID {
    static final int MAX_OBSERVATIONS = 5;
    double Kf = 0.0;
    double Kp = 0.0;
    double Kd = 0.0;
    double Ki = 0.0;
    double target = 0.0;
    XXCircularArray<Observation> observations;

    private Observation addObeservation(double actual) {
        double error = actual - target;

        Observation lastObservation = observations.size() > 0 ? observations.getLast() : null;
        double currentTime = System.currentTimeMillis();
        Observation currentObservation = new Observation(
                error,
                currentTime,
                lastObservation == null ? 0.0 : (currentTime - lastObservation.time) / 1000.0);

        observations.append(currentObservation);

        return currentObservation;
    }

    public XXSimplePID(double kf, double kp, double kd, double ki) {
        Kf = kf;
        Kp = kp;
        Kd = kd;
        Ki = ki;
        observations = new XXCircularArray<>(10);
    }

    public void setTarget(double target) {
        this.target = target;
        observations.clear();
    }

    public double getOutput(double actual) {
        Observation obs = addObeservation(actual);

        double sumi = 0.0;
        double avgd = 0.0;
        double prevError = 0.0;
        boolean firstIteration = true;
        for(Observation o : observations) {
            sumi += o.error * o.deltaTime;
            if (firstIteration) {
                firstIteration = false;
                prevError = o.error;
            } else {
                avgd = (o.error - prevError) / o.deltaTime;
            }
        }

        if(observations.size() > 1) {
            avgd = avgd / (observations.size() - 1);
        } else {
            avgd = 0.0;
        }

        double p = Kp * obs.error;
        double i = Ki * sumi;
        double d = Kd * avgd;
        double f = Kf * target;

        return p + i + d + f;
    }
}
