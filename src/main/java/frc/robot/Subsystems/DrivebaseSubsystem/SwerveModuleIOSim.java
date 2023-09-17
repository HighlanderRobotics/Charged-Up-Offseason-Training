// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DrivebaseSubsystem;

import java.util.function.DoubleSupplier;

import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.controls.ControlRequest;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

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
    
        inputs.swervePositionMeters = 0.0;
        inputs.drivePositionMeters = lastPosition;
        

        return inputs;
       

        


    }

    

    @Override
    public void setDrive(double rotation, double position) {
        

        lastPosition = position;
        lastRotation = rotation;

    }
}
