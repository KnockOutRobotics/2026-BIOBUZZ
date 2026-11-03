package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

/**
 * Live-tunable turret PID, shown in FTC Dashboard as "TurretTuning". {@link Turret} re-reads these
 * every loop. Copy the tuned values back here so they survive a redeploy. Fixed turret constants
 * (limits, ticks) stay in {@link SubsystemConstants.Turret}.
 */
@Config
public class TurretTuning {

    private TurretTuning() {
    }

    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.0; // static friction: F * sign(error), off inside tolerance
    public static double I_LIMIT = 0.2; // most power the I term may add
    public static double TOLERANCE_DEGREES = 2;
    public static double MAX_POWER = 1.0; // PID + feedforward is clipped to +/- this

    // Velocity feedforward: power per degree/second of target motion. Tune with P at 0: spin the
    // robot in place while aiming and raise KV until the turret holds on the goal.
    public static double KV = 0.0;
    // Smoothing on the measured target velocity, 0..1: 1 = raw, lower = smoother but laggier.
    public static double VELOCITY_FILTER = 0.5;
}
