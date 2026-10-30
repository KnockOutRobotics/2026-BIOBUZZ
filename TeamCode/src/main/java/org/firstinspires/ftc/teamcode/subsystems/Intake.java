package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.ivy.Subsystem;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake implements Subsystem {
  private final DcMotorEx m_Pollen;
  private final DcMotorEx m_Nectar;
  private final Servo s_indexer;

  public Intake(HardwareMap hm) {
    m_Pollen = hm.get(DcMotorEx.class, "pollenMotor");
    m_Nectar = hm.get(DcMotorEx.class, "nectarMotor");

    s_indexer = hm.get(Servo.class, "indexerServo");
  }

  // =========================================================
  // Ivy commands
  //
  // Each command requires the motor it drives rather than the whole subsystem, so the two
  // intakes run independently: holding nectar does not interrupt pollen. They run until
  // cancelled (bind with whileHeld) and stop their motor however they end.
  // =========================================================

  /** Runs the nectar intake until cancelled. */
  public CommandBuilder runNectar() {
    return run(m_Nectar, SubsystemConstants.Intake.NECTAR_POWER);
  }

  /** Runs the pollen intake until cancelled. */
  public CommandBuilder runPollen() {
    return run(m_Pollen, SubsystemConstants.Intake.POLLEN_POWER);
  }

  /** Runs both intakes until cancelled; interrupts either one already running on its own. */
  public CommandBuilder runBoth() {
    return Command.build()
        .requiring(m_Nectar, m_Pollen)
        .setStart(() -> {
          m_Nectar.setPower(SubsystemConstants.Intake.NECTAR_POWER);
          m_Pollen.setPower(SubsystemConstants.Intake.POLLEN_POWER);
        })
        .setDone(() -> false)
        .setEnd(endCondition -> {
          m_Nectar.setPower(0);
          m_Pollen.setPower(0);
        });
  }

  private static CommandBuilder run(DcMotorEx motor, double power) {
    return Command.build()
        .requiring(motor)
        .setStart(() -> motor.setPower(power))
        .setDone(() -> false)
        .setEnd(endCondition -> motor.setPower(0));
  }

  @Override
  public void periodic() {
  }
}
