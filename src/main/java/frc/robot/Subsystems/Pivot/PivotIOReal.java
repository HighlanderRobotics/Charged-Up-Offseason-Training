package frc.robot.Subsystems.Pivot;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.TorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class PivotIOReal implements PivotIO{

    private TalonFX pivotMotor = new TalonFX(Constants.PIVOT_MOTOR_ID);
    private PositionVoltage motorRequest = new PositionVoltage(Constants.PIVOT_MOTOR_ID);

    
    private StatusSignalValue<Double> supplyVoltageSignal = pivotMotor.getDutyCycle();
    private StatusSignalValue<Double> position = pivotMotor.getRotorPosition();
    private StatusSignalValue<Double> velocity = pivotMotor.getRotorVelocity();
    private StatusSignalValue<Double> currentDraw = pivotMotor.getStatorCurrent();


    public PivotIOReal (){
        TalonFXConfiguration pivotConfig  = new TalonFXConfiguration();

        pivotConfig.Slot0.kP = 0;
        pivotConfig.Slot0.kD = 0;
        pivotConfig.Slot0.kI = 0;
    }

    @Override
    public void setPosition(double degrees) {
        pivotMotor.setControl(motorRequest.withPosition(Units.degreesToRotations(degrees)*Constants.PIVOT_GEAR_RATIO));    
    }

    @Override
    public void reset(double degrees){ 
        pivotMotor.setRotorPosition((Units.degreesToRotations(degrees))*Constants.PIVOT_GEAR_RATIO);
    }
   

    @Override
    public PivotIOInputsAutoLogged updateInputs() {
        // TODO Auto-generated method stub
        PivotIOInputsAutoLogged current = new PivotIOInputsAutoLogged();

        current.currentDrawAmps = currentDraw.refresh().getValue();
        current.positionDegrees = position.refresh().getValue();
        current.velocityRPM = velocity.refresh().getValue();
        current.motorOutputVolts = 12 * supplyVoltageSignal.getValue();

        return(current);
    }
}