package frc.robot.commands;

import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.control.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SS_Drivebase;

public class C_AutonomousDrive extends CommandBase {

  private SS_Drivebase drivebase = new SS_Drivebase();

  private Vector2 targetTranslation;
  private double targetRotation;
  private double dt = 0.0;

  private double rotationKP = 0.0;
  private double rotationKI = 0.0;
  private double rotationKD = 0.0;
  private PidConstants rotationPID = new PidConstants(rotationKP, rotationKI, rotationKD);
  private PidController rotationController = new PidController(rotationPID);
  private double translationSpeed;
  private double translationMinSpeed = 0.0;

  private double translationKP = 0.0;
  private double translationKI = 0.0;
  private double translationKD = 0.0;
  private PidConstants translationPID = new PidConstants(translationKP, translationKI, translationKD);
  private PidController translationController = new PidController(translationPID);
  private double rotationSpeed;
  private double rotationMinSpeed = 0.0;


  public C_AutonomousDrive(Vector2 targetTranslation, double targetRotation) {
    this.targetTranslation = targetTranslation;
    this.targetRotation = targetRotation;

    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    drivebase.resetPose();
  }

  @Override
  public void execute() {
    double currentTranslation = drivebase.getPose().translation.length;
    translationSpeed = translationController.calculate(currentTranslation, dt);
    Vector2 translation = targetTranslation.normal().scale(translationSpeed);

    rotationSpeed = rotationController.calculate(targetRotation, dt);

    drivebase.drive(translation, rotationSpeed, true);
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(Vector2.ZERO, 0.0, true);
  }

  @Override
  public boolean isFinished() {
    return ((translationSpeed < translationMinSpeed) && (rotationSpeed < rotationMinSpeed));
  }
}
