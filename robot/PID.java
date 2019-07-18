
package frc.robot;

/**
 * Pid
 */
public class PID {
    double kP;
    double kI;
    double i_zone;
    double kD;
    double kF;

    double prev_err;
    double i_state;

    public PID(double p, double i, double iz, double d, double f) {
        this.kP = p;
        this.kI = i;
        this.i_zone = iz;
        this.kD = d;
        this.kF = f;
    }
    
}