// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DrivebaseSubsystem;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.Timestamp;
import com.ctre.phoenixpro.Timestamp.TimestampSource;
import com.ctre.phoenixpro.hardware.Pigeon2;

/** Add your docs here. */
public class GyroModuleIOReal implements GyroModuleIO{
    Pigeon2 gyro;
    StatusSignalValue statusSignal;
    Timestamp dTime;
    double deltaPitch = gyro.getPitch().getValue();
    
    public GyroModuleIOReal(int gyroID){
        gyro = new Pigeon2(gyroID);
        statusSignal = new StatusSignalValue<>(null, null);
    }

    private double getAngularRate() {
        double slope = 0.0;
        if(dTime.getTime() != statusSignal.getTimestamp().getTime()){
            slope = (gyro.getPitch().getValue() - deltaPitch) / (statusSignal.getTimestamp().getTime() - dTime.getTime());

            dTime = statusSignal.getTimestamp();
            deltaPitch = gyro.getPitch().getValue();
        }
        
        
        
        return slope;
    }
    @Override
    public GyroModuleIOInputsAutoLogged updateInputs() {
        GyroModuleIOInputsAutoLogged inputs = new GyroModuleIOInputsAutoLogged();
        inputs.pitch = gyro.getPitch().getValue();
        inputs.roll = gyro.getRoll().getValue();
        inputs.yaw = gyro.getYaw().getValue();
        inputs.angularRate = getAngularRate();
        return inputs;
    }}

    
