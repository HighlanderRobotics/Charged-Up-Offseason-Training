// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Autonomous;
import frc.robot.Subsystems.DrivebaseSubsystem.SwerveModuleIOReal;
import frc.robot.Subsystems.DrivebaseSubsystem.SwerveModuleIOSim;
import frc.robot.Subsystems.DrivebaseSubsystem.SwerveSubsystem;

public class RobotContainer {
  CommandXboxController controller = new CommandXboxController(0);

  Autonomous autonomous = new Autonomous();


  SwerveSubsystem swerveSubsystem = new SwerveSubsystem(
    new SwerveModuleIOSim(), 
    new SwerveModuleIOSim(), 
    new SwerveModuleIOSim(), 
    new SwerveModuleIOSim());

  public RobotContainer() {
    configureBindings();
  }
  
  public double deadband(double joystick){
    if(Math.abs(joystick) < 0.1){
      return 0;
    }
    return joystick;
  }

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.drive(
      () -> deadband(controller.getLeftY()) * Constants.DRIVEBASE_MAX_SPEED_FPS * -1 /* invert controls */,
      () -> deadband(controller.getLeftX()) * Constants.DRIVEBASE_MAX_SPEED_FPS * -1 /* invert controls */, 
      () -> deadband(controller.getRightX()) * -1 /* invert controls */,
      true) );
      
  }

  public Command getAutonomousCommand() {
    return autonomous.auto(
      () -> 0.5,
      () -> 0,
      () -> 0,
      true
    );
  }
}
