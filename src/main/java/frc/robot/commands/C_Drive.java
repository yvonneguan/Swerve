package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.XboxController;

import frc.robot.RobotContainer;
import frc.robot.subsystems.SS_Drivebase;

public class C_Drive extends CommandBase {

  private SS_Drivebase drivebase = SS_Drivebase.getInstance();
  private XboxController controller = RobotContainer.getController();

  public C_Drive() {
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double forward = controller.getLeftYAxis().get();
    double strafe = controller.getLeftXAxis().get();
    Vector2 translationV = new Vector2(forward, strafe);

    double rotation = controller.getRightXAxis().get();

    drivebase.drive(translationV, rotation, true);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      drivebase.drive(Vector2.ZERO, 0.0, true);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
