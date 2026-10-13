package org.firstinspires.ftc.teamcode.ivy;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * Base class for every OpMode in this project. Owns the Ivy {@link Scheduler} loop so OpModes only
 * describe <em>what</em> should run.
 *
 * <p>The lifecycle, in order:
 *
 * <ol>
 * <li>{@link #onInit()} -- map hardware and {@link #register(Subsystem)} subsystems. Nothing is
 *     scheduled and no {@code periodic()} runs yet, so the robot stays still on INIT.</li>
 * <li>{@link #onInitLoop()} -- repeated until START (vision reads, alliance selection).</li>
 * <li>Each registered subsystem's {@code periodic()} is scheduled as a requirement-free Ivy
 *     command, and each {@code defaultCommand()} at {@link #DEFAULT_COMMAND_PRIORITY}.</li>
 * <li>{@link #onStart()} -- schedule the autonomous routine or set up {@link #bindings}.</li>
 * <li>{@link #onLoop()} then {@link Scheduler#execute()}, once per loop, until STOP. Button
 *     bindings are polled first, so a command scheduled by a press runs on the same loop.</li>
 * <li>{@link #onStop()}, then the scheduler is cleared.</li>
 * </ol>
 *
 * <p>Telemetry is pushed once per loop after {@link #onLoop()}; subclasses only need
 * {@code telemetry.addData(...)}.
 */
public abstract class IvyOpMode extends LinearOpMode {

    /**
     * Priority given to default commands. Below the Ivy default of 0, so any ordinary command
     * interrupts the default, and a default that is waiting to come back is dropped (rather than
     * stealing the subsystem) while a real command holds it.
     */
    public static final int DEFAULT_COMMAND_PRIORITY = -1;

    /** Button-to-command bindings, polled once per loop. */
    protected final Bindings bindings = new Bindings();

    private final List<Subsystem> subsystems = new ArrayList<>();
    private final List<Command> defaultCommands = new ArrayList<>();

    /** Map hardware and register subsystems. Runs once, before INIT finishes. */
    protected abstract void onInit();

    /** Runs repeatedly between INIT and START. */
    protected void onInitLoop() {
    }

    /** Runs once at START, before the first scheduler loop. */
    protected void onStart() {
    }

    /** Runs every loop, just before {@link Scheduler#execute()}. */
    protected void onLoop() {
    }

    /** Runs once after the OpMode loop exits, before the scheduler is cleared. */
    protected void onStop() {
    }

    /**
     * Registers a subsystem so its {@code periodic()} and {@code defaultCommand()} are run by the
     * scheduler. Returns the subsystem, so it can be used inline:
     * {@code swerve = register(new SwerveDrive(telemetry, hardwareMap));}
     */
    protected final <T extends Subsystem> T register(T subsystem) {
        subsystems.add(subsystem);
        return subsystem;
    }

    /** Schedules commands. Shorthand for {@link Scheduler#schedule(Command...)}. */
    protected final void schedule(Command... commands) {
        Scheduler.schedule(commands);
    }

    /**
     * Runs {@code command} whenever nothing else requires the subsystems it needs, the same way a
     * {@link Subsystem#defaultCommand()} is run. Use this for a default that belongs to the OpMode
     * rather than the subsystem -- stick drive, for instance, which needs the gamepads. Call it
     * from {@link #onInit()} or {@link #onStart()}.
     */
    protected final void setDefaultCommand(Command command) {
        defaultCommands.add(new DefaultCommand(command));
    }

    @Override
    public final void runOpMode() throws InterruptedException {
        // Ivy's scheduler is static, so commands from the previous OpMode would otherwise carry over.
        Scheduler.reset();

        // One bulk read per hub per loop, instead of one bus transaction per encoder or analog
        // read. The cache is cleared at the top of every loop, so values are never more than one
        // loop old.
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        onInit();
        telemetry.update();

        while (opModeInInit()) {
            clearBulkCache(hubs);
            onInitLoop();
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) {
            Scheduler.reset();
            return;
        }

        for (Subsystem subsystem : subsystems) {
            Scheduler.schedule(Commands.infinite(subsystem::periodic));

            Command defaultCommand = subsystem.defaultCommand();
            if (defaultCommand != null) {
                defaultCommands.add(new DefaultCommand(defaultCommand));
            }
        }

        onStart();

        while (opModeIsActive()) {
            clearBulkCache(hubs);
            bindings.poll();

            // A default whose subsystem is busy is blocked and dropped, and tried again next loop.
            for (Command defaultCommand : defaultCommands) {
                if (!Scheduler.isScheduled(defaultCommand)) {
                    Scheduler.schedule(defaultCommand);
                }
            }

            onLoop();
            Scheduler.execute();
            telemetry.update();
        }

        onStop();
        Scheduler.reset();
    }

    private static void clearBulkCache(List<LynxModule> hubs) {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    /**
     * A subsystem's default command, re-prioritised so it yields to everything else. Delegates the
     * four lifecycle methods and the requirements to the command the subsystem handed us.
     */
    private static final class DefaultCommand implements Command {

        private final Command command;

        private DefaultCommand(Command command) {
            this.command = command;
        }

        @Override
        public Set<Object> requirements() {
            return command.requirements();
        }

        @Override
        public int priority() {
            return DEFAULT_COMMAND_PRIORITY;
        }

        @Override
        public InterruptedBehavior interruptedBehavior() {
            return InterruptedBehavior.END;
        }

        @Override
        public ConflictBehavior conflictBehavior() {
            return ConflictBehavior.CANCEL;
        }

        @Override
        public BlockedBehavior blockedBehavior() {
            return BlockedBehavior.CANCEL;
        }

        @Override
        public void start() {
            command.start();
        }

        @Override
        public boolean done() {
            return command.done();
        }

        @Override
        public void execute() {
            command.execute();
        }

        @Override
        public void end(EndCondition endCondition) {
            command.end(endCondition);
        }
    }
}
