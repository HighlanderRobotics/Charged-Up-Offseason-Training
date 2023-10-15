// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.DrivebaseSubsystem.SwerveModuleIOSim;
import frc.robot.Subsystems.DrivebaseSubsystem.SwerveSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

/** Add your docs here. */
public class Autonomous {
    
    SwerveSubsystem swerveSubsystem;
    IntakeSubsystem intakeSubsystem;
    LoggedDashboardChooser<Command> autoSystemType =
    new LoggedDashboardChooser<>("Auto System Type");

    public Autonomous(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        autoSystemType.addDefaultOption("Balance", chargeStation());
    }
    
    public Command getAutoCommand(){
        return autoSystemType.get();
    }

    public CommandBase chargeStation(){
        /*
         * score piece
         * move backward
         * balance on charging station
         */

         return Commands.sequence(
            intakeSubsystem.outtake(),
            swerveSubsystem.drive(() -> -2, () -> 0, () -> 0, false).withTimeout(2),
            swerveSubsystem.balance()
            );
        
    }


}
