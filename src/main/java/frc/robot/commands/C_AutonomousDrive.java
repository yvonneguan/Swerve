package frc.robot.commands;

import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.control.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SS_Drivebase;

public class C_AutonomousDrive extends CommandBase {

  private SS_Drivebase drivebase = new SS_Drivebase();

  private Vector2 translation;
  private double rotation;
  private double dt = 0.0;

  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;

  private PidConstants pidConstants = new PidConstants(kP, kI, kD);
  private PidController pidController = new PidController(pidConstants);

  public C_AutonomousDrive(Vector2 translation, double rotation) {
    this.translation = translation;
    this.rotation = rotation;

    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    drivebase.resetPose();
  }

  @Override
  public void execute() {
    Vector2 currentTranslation = drivebase.getPose().translation;
    double kTranslation = pidController.calculate();

    drivebase.drive(translation, rotation, true);
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(Vector2.ZERO, 0.0, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
