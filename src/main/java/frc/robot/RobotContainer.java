// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.DrivebaseSubsystem.SwerveModuleIOReal;
import frc.robot.Subsystems.DrivebaseSubsystem.SwerveModuleIOSim;
import frc.robot.Subsystems.DrivebaseSubsystem.SwerveSubsystem;

public class RobotContainer {
  CommandXboxController controller = new CommandXboxController(0);


  SwerveSubsystem swerveSubsystem = new SwerveSubsystem(
    new SwerveModuleIOSim(), 
    new SwerveModuleIOSim(), 
    new SwerveModuleIOSim(), 
    new SwerveModuleIOSim());

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.drive(
      () -> controller.getLeftY() * Constants.DRIVEBASE_MAX_SPEED_FPS * -1 /* invert controls */,
      () -> controller.getLeftX() * Constants.DRIVEBASE_MAX_SPEED_FPS , 
      () -> controller.getRightX() * -1 /* invert controls */) );
      
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
