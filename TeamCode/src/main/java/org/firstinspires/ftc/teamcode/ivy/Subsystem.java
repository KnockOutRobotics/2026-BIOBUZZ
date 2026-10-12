package org.firstinspires.ftc.teamcode.ivy;

import com.pedropathing.ivy.Command;

/**
 * A piece of robot hardware that {@link IvyOpMode} hands to the Ivy scheduler.
 *
 * <p>Ivy has no subsystem type of its own -- a command's requirements are plain objects, so the
 * subsystem instance itself is the requirement. This interface only adds the two hooks an OpMode
 * needs to run hardware without the OpMode loop touching it directly:
 *
 * <ul>
 * <li>{@link #periodic()} -- the closed loop. {@link IvyOpMode} schedules it as a
 *     requirement-free Ivy command, so it keeps running no matter which command currently owns
 *     the subsystem. Read sensors and write motor powers here; never block.</li>
 * <li>{@link #defaultCommand()} -- what the subsystem does when nothing else wants it (driving
 *     from the sticks, holding a position). {@link IvyOpMode} keeps it scheduled at a lower
 *     priority than everything else, so any ordinary command interrupts it and it comes back on
 *     its own once that command finishes.</li>
 * </ul>
 *
 * <p>Commands that move the subsystem belong on the subsystem itself as factory methods that
 * return {@code Command.build().requiring(this)...}, so two commands can never fight over the
 * same motors.
 */
public interface Subsystem {

    /** Runs once per scheduler loop, for the whole OpMode. Does nothing by default. */
    default void periodic() {
    }

    /**
     * The command to run whenever no other command requires this subsystem, or {@code null} for
     * none. Called once, after start; the returned command must require this subsystem to be
     * handed back correctly after an interruption.
     */
    default Command defaultCommand() {
        return null;
    }
}
