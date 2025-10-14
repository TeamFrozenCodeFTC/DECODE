package org.firstinspires.ftc.blackice.core.control;


@FunctionalInterface
public interface Feedforward {
    double compute(double target);
    
    static Feedforward zero() {
        return target -> 0;
    }
    
    class Linear implements Feedforward {
        double kV, kS;
        
        public Linear(double kV, double kS) {
            this.kV = kV;
            this.kS = kS;
        }
        
        @Override
        public double compute(double target) {
            return kV * target + kS * Math.signum(target);
        }
    }
}
