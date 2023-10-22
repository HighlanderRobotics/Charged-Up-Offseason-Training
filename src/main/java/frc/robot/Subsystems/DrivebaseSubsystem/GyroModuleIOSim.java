// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DrivebaseSubsystem;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.Timestamp;
import com.ctre.phoenixpro.Timestamp.TimestampSource;
import com.ctre.phoenixpro.hardware.Pigeon2;

/** Add your docs here. */
public class GyroModuleIOSim implements GyroModuleIO{
    Pigeon2 gyro;
    StatusSignalValue statusSignal;
    Timestamp dTime;
    
    
    public GyroModuleIOSim(){
       
        statusSignal = new StatusSignalValue<>(null, null);
    }

    private double getAngularRate() {
        double slope = 0.0;
        
        
        
        
        return slope;
    }
    @Override
    public GyroModuleIOInputsAutoLogged updateInputs() {
        GyroModuleIOInputsAutoLogged inputs = new GyroModuleIOInputsAutoLogged();
        inputs.pitch = 0;
        inputs.roll = 0;
        inputs.yaw = 0;
        inputs.angularRate = 0;
        return inputs;
    }}

    
