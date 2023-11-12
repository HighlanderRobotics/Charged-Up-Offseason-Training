// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DrivebaseSubsystem;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

/** Add your docs here. */
public class GyroModuleIOReal implements GyroModuleIO{
    Pigeon2 gyro;
    StatusSignal<Double> statusSignal;
    Double dTime;
    double deltaPitch;
    
    public GyroModuleIOReal(int gyroID){
        gyro = new Pigeon2(gyroID);
        statusSignal = gyro.getPitch();
        dTime = statusSignal.getValue();
        deltaPitch = 0;
    }

    private double getAngularRate() {
        double slope = 0.0;
        if(dTime != statusSignal.getValue()){
            slope = (gyro.getPitch().getValue() - deltaPitch) / (statusSignal.getValue() - dTime);

            dTime = statusSignal.getValue();
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

    
