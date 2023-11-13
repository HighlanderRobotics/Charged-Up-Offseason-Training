// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DrivebaseSubsystem;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class SwerveModuleIOSim implements SwerveModuleIO{

    double lastRotation;
    double lastPosition;

    VoltageOut swerveVoltage = new VoltageOut(0);
    VoltageOut driveVoltage = new VoltageOut(0);


    PositionVoltage swerveRequest = new PositionVoltage(0);
    VelocityVoltage driveRequest = new VelocityVoltage(0);

    double driveVelocity;


    public SwerveModuleIOSim(){
        
    }

    @Override
    public SwerveModuleIOInputsAutoLogged updateInputs() {

        SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    
        inputs.swerveVelocityMetersPerSecond = 0.0;
        inputs.driveVelocityMetersPerSecond = 0.0;
    
        inputs.swerveRotationRotations = lastRotation;
        inputs.drivePositionMeters = lastPosition;
        

        return inputs;
       

        


    }

    

    @Override
    public void setDrive(Rotation2d rotation, double position) {
        

        lastPosition += position * 0.02;
      //  lastRotation += rotation * 0.02;

    }

    @Override
    public void setDriveVoltage(double rotation, double driveVoltage) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void resetEncoder(){
        
    }
}
