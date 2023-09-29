// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.DrivebaseSubsystem.SwerveModuleIOSim;
import frc.robot.Subsystems.DrivebaseSubsystem.SwerveSubsystem;

/** Add your docs here. */
public class Autonomous {
    SwerveSubsystem swerveSubsystem;

    public Autonomous(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
    }

    public CommandBase auto(){
        /*
         * score piece
         * move backward
         * balance on charging station
         */

         return Commands.sequence(
            swerveSubsystem.drive(() -> 2, () -> 0, () -> 0, false).withTimeout(2)
            );
        
    }


}
