// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.XboxController;
import frc.robot.commands.C_Drive;
import frc.robot.subsystems.SS_Drivebase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {

  Vector2 autoDriveTargetTranslation;
  double autoDriveTargetRotation;

  public final SS_Drivebase drivebase = SS_Drivebase.getInstance();
  
  private static final XboxController controller = new XboxController(Constants.XBOXCONTROLLER_ID);

  public RobotContainer() {
    CommandScheduler.getInstance().setDefaultCommand(drivebase, new C_Drive());
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    controller.getBackButton().whenPressed(new InstantCommand(() -> drivebase.resetGyroAngle(Rotation2.ZERO), drivebase));
  }

  /*
  public Command getAutonomousCommand() {
    return drive;
  }
  */

  public static XboxController getController() {
    return controller;
  }
}
