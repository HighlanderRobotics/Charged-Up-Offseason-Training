// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.DrivebaseSubsystem.SwerveModuleIOSim;
import frc.robot.Subsystems.DrivebaseSubsystem.SwerveSubsystem;

public class RobotContainer {
  
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem(
    new SwerveModuleIOSim(0, 1), 
    new SwerveModuleIOSim(2, 3), 
    new SwerveModuleIOSim(4, 5), 
    new SwerveModuleIOSim(6, 7));

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
