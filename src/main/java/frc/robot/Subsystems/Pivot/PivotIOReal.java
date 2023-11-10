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

    public static final TalonFX pivotMotor = new TalonFX(Constants.PIVOT_MOTOR_ID);
    private StatusSignalValue<Double> supplyVoltageSignal = pivotMotor.getDutyCycle();
    private PositionVoltage motorRequest = new PositionVoltage(0);
    private StatusSignalValue<Double> position = pivotMotor.getRotorPosition();
    private StatusSignalValue<Double> velocity = pivotMotor.getRotorVelocity();
    private StatusSignalValue<Double> currentDraw = pivotMotor.getStatorCurrent();

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

        current.currentDrawAmps = currentDraw.getValue();
        current.positionDegrees = position.getValue();
        current.velocityRPM = velocity.getValue();
        current.motorOutputVolts = 12 * supplyVoltageSignal.getValue();

        return(current);
    }
}