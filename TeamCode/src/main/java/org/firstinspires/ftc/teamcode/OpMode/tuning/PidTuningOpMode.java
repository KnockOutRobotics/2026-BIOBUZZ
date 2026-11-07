package org.firstinspires.ftc.teamcode.OpMode.tuning;

import java.util.ArrayList;
import java.util.List;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.tuning.TuningStep;

/**
 * Shared shell for the PID tuners (procedures in TUNING.md next to this file). A tuner is a list of
 * {@link TuningStep}s, run one at a time from gamepad 1:
 *
 * <ul>
 * <li>dpad up/down -- pick a step (while none is running)</li>
 * <li>A -- start the picked step</li>
 * <li>B -- stop it (the mechanism goes limp)</li>
 * </ul>
 *
 * <p>Gains are changed live on FTC Dashboard while a step runs; telemetry goes to both the Driver
 * Station and Dashboard, where target/measured/error/power can be graphed.
 */
public abstract class PidTuningOpMode extends LinearOpMode {

    protected Telemetry tele;

    private final List<TuningStep> steps = new ArrayList<>();
    private int selected = 0;
    private TuningStep running;
    private final ElapsedTime clock = new ElapsedTime();

    private boolean lastUp;
    private boolean lastDown;
    private boolean lastA;
    private boolean lastB;

    /** Shown at the top of the screen. */
    protected abstract String title();

    /** Map hardware and {@link #addStep} the steps, in order. */
    protected abstract void initHardware();

    /** Read sensors, once per loop before the step runs. */
    protected abstract void updateSensors(double now);

    /** Cut all power. Called every loop while no step runs, so the mechanism can be moved by hand. */
    protected abstract void stopOutputs();

    /** Live values and current gains, shown under the step. */
    protected abstract void addStatus(Telemetry telemetry);

    /** Tuner-specific buttons, every loop after the step. */
    protected void extraControls(double now) {
    }

    protected final void addStep(TuningStep step) {
        steps.add(step);
    }

    protected final boolean isStepRunning() {
        return running != null;
    }

    @Override
    public final void runOpMode() {
        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        initHardware();
        clock.reset();

        while (opModeInInit()) {
            clearBulkCache(hubs);
            updateSensors(clock.seconds());
            show();
        }

        while (opModeIsActive()) {
            clearBulkCache(hubs);
            double now = clock.seconds();
            updateSensors(now);
            handleButtons(now);

            if (running != null) {
                running.loop(now);
                if (running.isDone()) {
                    finishStep();
                }
            } else {
                stopOutputs();
            }

            extraControls(now);
            show();
        }

        if (running != null) {
            finishStep();
        }
        stopOutputs();
    }

    private void handleButtons(double now) {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;

        if (running == null) {
            if (up && !lastUp) {
                selected = (selected + steps.size() - 1) % steps.size();
            }
            if (down && !lastDown) {
                selected = (selected + 1) % steps.size();
            }
            if (a && !lastA) {
                running = steps.get(selected);
                running.start(now);
            }
        } else if (b && !lastB) {
            finishStep();
        }

        lastUp = up;
        lastDown = down;
        lastA = a;
        lastB = b;
    }

    private void finishStep() {
        running.end();
        running = null;
        stopOutputs();
    }

    private void show() {
        tele.addLine(title() + " | dpad: pick, A: start, B: stop");
        for (int i = 0; i < steps.size(); i++) {
            TuningStep step = steps.get(i);
            tele.addLine((i == selected ? "> " : "   ") + step.name() + (step == running ? "  [RUNNING]" : ""));
        }
        tele.addLine("");

        TuningStep step = steps.get(selected);
        for (String line : step.instructions()) {
            tele.addLine(line);
        }
        tele.addLine("");
        step.report(tele, step == running);

        tele.addLine("");
        addStatus(tele);
        tele.update();
    }

    private static void clearBulkCache(List<LynxModule> hubs) {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}
