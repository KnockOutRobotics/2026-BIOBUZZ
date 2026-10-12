package org.firstinspires.ftc.teamcode.ivy;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * Button-to-command bindings, with edge detection.
 *
 * <p>Ivy schedules and composes commands but has no trigger type, so this is the missing half:
 * {@link IvyOpMode} calls {@link #poll()} once per loop and each binding schedules (or cancels)
 * its command when its button changes state. Bindings read the gamepad through a
 * {@link BooleanSupplier}, so they are written as method references to the live gamepad fields:
 *
 * <pre>{@code
 * bindings.whenPressed(() -> gamepad1.options, swerve.resetIMUCommand())
 *         .whileHeld(() -> gamepad1.right_bumper, turret.goToAngle(90));
 * }</pre>
 *
 * <p>Pass a command instance per binding; scheduling the same instance from two bindings would
 * share its state.
 */
public final class Bindings {

    private enum Edge {
        /** Schedule on the press. */
        PRESSED,
        /** Schedule on the release. */
        RELEASED,
        /** Schedule on the press, cancel on the release. */
        HELD
    }

    private static final class Binding {

        private final BooleanSupplier button;
        private final Edge edge;
        private final Command command;
        private boolean wasPressed;

        private Binding(BooleanSupplier button, Edge edge, Command command) {
            this.button = button;
            this.edge = edge;
            this.command = command;
        }
    }

    private final List<Binding> bindings = new ArrayList<>();

    /** Schedules {@code command} once, each time the button goes down. */
    public Bindings whenPressed(BooleanSupplier button, Command command) {
        return add(button, Edge.PRESSED, command);
    }

    /** Schedules {@code command} once, each time the button comes back up. */
    public Bindings whenReleased(BooleanSupplier button, Command command) {
        return add(button, Edge.RELEASED, command);
    }

    /**
     * Schedules {@code command} while the button is held and cancels it on release, so an
     * infinite command (jogging a mechanism) stops when the driver lets go. The command's
     * {@code end(INTERRUPTED)} runs on the release.
     */
    public Bindings whileHeld(BooleanSupplier button, Command command) {
        return add(button, Edge.HELD, command);
    }

    /** Reads every button and acts on the ones that changed. Called once per loop. */
    public void poll() {
        for (Binding binding : bindings) {
            boolean pressed = binding.button.getAsBoolean();

            if (pressed && !binding.wasPressed) {
                if (binding.edge == Edge.PRESSED || binding.edge == Edge.HELD) {
                    Scheduler.schedule(binding.command);
                }
            } else if (!pressed && binding.wasPressed) {
                if (binding.edge == Edge.RELEASED) {
                    Scheduler.schedule(binding.command);
                } else if (binding.edge == Edge.HELD) {
                    Scheduler.cancel(binding.command);
                }
            }

            binding.wasPressed = pressed;
        }
    }

    /** Drops every binding. The commands they scheduled keep running. */
    public void clear() {
        bindings.clear();
    }

    private Bindings add(BooleanSupplier button, Edge edge, Command command) {
        bindings.add(new Binding(button, edge, command));
        return this;
    }
}
