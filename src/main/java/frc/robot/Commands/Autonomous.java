// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.DrivebaseSubsystem.SwerveModuleIOSim;
import frc.robot.Subsystems.DrivebaseSubsystem.SwerveSubsystem;

/** Add your docs here. */
public class Autonomous {
    SwerveSubsystem swerveSubsystem = new SwerveSubsystem(
    new SwerveModuleIOSim(), 
    new SwerveModuleIOSim(), 
    new SwerveModuleIOSim(), 
    new SwerveModuleIOSim());

    public  CommandBase auto(DoubleSupplier forward, DoubleSupplier turn, DoubleSupplier theta, boolean isFieldRelative){
        return new RunCommand(() -> {
            swerveSubsystem.drive(
                () -> forward.getAsDouble(),
                () -> turn.getAsDouble(), 
                () -> theta.getAsDouble(),
                isFieldRelative);

        });
        
    }
}
