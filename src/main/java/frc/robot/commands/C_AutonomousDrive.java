package frc.robot.commands;

import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.control.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SS_Drivebase;

public class C_AutonomousDrive extends CommandBase {

  private SS_Drivebase drivebase = SS_Drivebase.getInstance();

  private Vector2 targetTranslation;
  private double targetRotation;

  private double time = 0;

  private double rotationKP = 0.0;
  private double rotationKI = 0.0;
  private double rotationKD = 0.0;
  private PidConstants rotationPID = new PidConstants(rotationKP, rotationKI, rotationKD);
  private PidController rotationController = new PidController(rotationPID);
  private double translationSpeed = 0.0;
  private double translationMinSpeed = 0.0;

  private double translationKP = 0.0;
  private double translationKI = 0.0;
  private double translationKD = 0.0;
  private PidConstants translationPID = new PidConstants(translationKP, translationKI, translationKD);
  private PidController translationController = new PidController(translationPID);
  private double rotationSpeed = 0.0;
  private double rotationMinSpeed = 0.0;


  public C_AutonomousDrive(Vector2 targetTranslation, double targetRotation) {
    this.targetTranslation = targetTranslation;
    this.targetRotation = targetRotation;

// new
    translationController.setOutputRange(-1.0, 1.0);
    translationController.setSetpoint(targetTranslation.length);

    rotationController.setOutputRange(-1.0, 1.0);
    rotationController.setInputRange(0, 2 * Math.PI);
    rotationController.setContinuous(true);
    rotationController.setSetpoint(targetRotation);

    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
// When would we need multiple autonomous commands? What variable is it that I need to reset?
    drivebase.resetPose();
  }

  @Override
  public void execute() {
// new
    double currentTime = Timer.getFPGATimestamp();
    time = currentTime;
    double dt = currentTime - time;

    double currentTranslation = drivebase.getPose().translation.length;
    translationSpeed = translationController.calculate(currentTranslation, dt);
    Vector2 translation = targetTranslation.normal().scale(translationSpeed);

    double currentRotation = drivebase.getPose().rotation.toRadians();
    rotationSpeed = rotationController.calculate(currentRotation, dt);

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
