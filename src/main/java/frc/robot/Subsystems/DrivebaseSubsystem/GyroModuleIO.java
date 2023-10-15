// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DrivebaseSubsystem;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface GyroModuleIO {

    @AutoLog
    public static class GyroModuleIOInputs{
        public double pitch = 0.0;
        public double yaw = 0.0;
        public double roll = 0.0;
        public double angularRate = 0.0;
    }



    public abstract GyroModuleIOInputsAutoLogged updateInputs();
}
