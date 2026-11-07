package org.firstinspires.ftc.teamcode.util.tuning;

/** Runs a mechanism's real closed loop for one cycle, toward {@code target}. */
@FunctionalInterface
public interface ClosedLoop {

    /**
     * @param target     setpoint, in the mechanism's units
     * @param targetRate how fast the setpoint is moving (units/s), for velocity feedforward; 0 for a
     *                   fixed setpoint
     */
    void run(double target, double targetRate);
}
