// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DrivebaseSubsystem;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double swerveOutputVolts = 0.0;
        public double driveOutputVolts = 0.0;
    
        public double swerveVelocityMetersPerSecond = 0.0;
        public double driveVelocityMetersPerSecond = 0.0;
    
        public double swerveRotationRotations = 0.0;
        public double drivePositionMeters = 0.0;

        public double encoderPosition;
    
        public double[] swerveCurrentAmps = new double[0];
        public double[] swerveTempCelsius = new double[0];
        public double[] driveCurrentAmps = new double[0];
        public double[] driveTempCelsius = new double[0];

        

        

        
    }


    public abstract SwerveModuleIOInputsAutoLogged updateInputs();

    public abstract void setDrive(Rotation2d rotation, double driveMPS);

    public abstract void setDriveVoltage(double rotation, double driveVoltage);

    public abstract void resetEncoder();
}
