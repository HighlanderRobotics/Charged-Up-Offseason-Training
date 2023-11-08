// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.TorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class PivotIOReal implements PivotIO{

    private static final StatusSignalValue<Double> supplyVoltageSignal = null;
    private final PositionVoltage motorRequest = new PositionVoltage(0);
    private StatusSignalValue<Double> position = Constants.PIVOT_MOTOR.getRotorPosition();
    private StatusSignalValue<Double> velocity = Constants.PIVOT_MOTOR.getRotorVelocity();
    private StatusSignalValue<Double> currentDraw = Constants.PIVOT_MOTOR.getStatorCurrent();

    @Override
    public void setPosition(double degrees) {
        Constants.PIVOT_MOTOR.setControl(motorRequest.withPosition(Units.degreesToRotations(degrees)*Constants.PIVOT_GEAR_RATIO));    
    }

    @Override
    public void reset(double degrees){ 
        Constants.PIVOT_MOTOR.setRotorPosition((Units.degreesToRotations(degrees))*Constants.PIVOT_GEAR_RATIO);
    }
   

    @Override
    public PivotIOInputsAutoLogged updateInputs() {
        // TODO Auto-generated method stub
        PivotIOInputsAutoLogged current = new PivotIOInputsAutoLogged();
        double supplyVoltage = supplyVoltageSignal.getValue();
    }
}