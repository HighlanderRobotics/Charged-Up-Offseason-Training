// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenixpro.hardware.CANcoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Autonomous;
import frc.robot.Subsystems.DrivebaseSubsystem.GyroModuleIOReal;
import frc.robot.Subsystems.DrivebaseSubsystem.SwerveModuleIOReal;
import frc.robot.Subsystems.DrivebaseSubsystem.SwerveModuleIOSim;
import frc.robot.Subsystems.DrivebaseSubsystem.SwerveSubsystem;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeIOReal;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Pivot.PivotIOReal;
import frc.robot.Subsystems.Pivot.PivotSubsystem;

public class RobotContainer {
  CommandXboxController controller = new CommandXboxController(0);

  


  // SwerveSubsystem swerveSubsystem = new SwerveSubsystem(

  //   Robot.isReal() ? new SwerveModuleIOReal(
  //     Constants.MOTOR_ID_FRONT_LEFT_ROTATION,
  //     Constants.MOTOR_ID_FRONT_LEFT_DRIVE,
  //     Constants.ENCODER_ID_FRONT_LEFT) : new SwerveModuleIOSim(), 
  //   Robot.isReal() ? new SwerveModuleIOReal(
  //     Constants.MOTOR_ID_FRONT_RIGHT_ROTATION, 
  //     Constants.MOTOR_ID_FRONT_RIGHT_DRIVE, 
  //     Constants.ENCODER_ID_FRONT_RIGHT) : new SwerveModuleIOSim(), 
  //   Robot.isReal() ? new SwerveModuleIOReal(
  //     Constants.MOTOR_ID_BACK_LEFT_ROTATION, 
  //     Constants.MOTOR_ID_BACK_LEFT_DRIVE, 
  //     Constants.ENCODER_ID_BACK_LEFT) : new SwerveModuleIOSim(), 
  //   Robot.isReal() ? new SwerveModuleIOReal(
  //     Constants.MOTOR_ID_BACK_RIGHT_ROTATION, 
  //     Constants.MOTOR_ID_BACK_RIGHT_DRIVE, 
  //     Constants.ENCODER_ID_BACK_RIGHT) : new SwerveModuleIOSim(),
  //   new GyroModuleIOReal(Constants.GYRO_MODULE_ID));

  IntakeSubsystem intakeSubsystem = new IntakeSubsystem(new IntakeIOReal());
  PivotSubsystem pivotSubsystem = new PivotSubsystem(new PivotIOReal());
  
  public RobotContainer() {
    configureBindings();
  }
//  Autonomous autonomous = new Autonomous(swerveSubsystem,intakeSubsystem);
  public double deadband(double joystick){
    if(Math.abs(joystick) < 0.1){
      return 0;
    }
    return joystick;
  }

  private void configureBindings() {
    // swerveSubsystem.setDefaultCommand(swerveSubsystem.drive(
    //   () -> deadband(controller.getLeftY()) * Constants.DRIVEBASE_MAX_SPEED_FPS * -1 /* invert controls */,
    //   () -> deadband(controller.getLeftX()) * Constants.DRIVEBASE_MAX_SPEED_FPS * -1 /* invert controls */, 
    //   () -> deadband(controller.getRightX()) * -1 /* invert controls */,
    //   true) );
      
  }

  public Command getAutonomousCommand() {
   // return autonomous.getAutoCommand();
   return new InstantCommand();
  }
}
