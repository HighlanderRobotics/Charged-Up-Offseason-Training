// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.TorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.math.util.Units;

public class PivotIOReal implements PivotIO{

    private static final double gearRatio = 1;
    private static final String BaseStatusSignal = null;
    private static final StatusSignalValue<Double> supplyVoltageSignal = null;
    private final TalonFX motor = new TalonFX(0);
    private final PositionVoltage motorRequest = new PositionVoltage(0);
    private StatusSignalValue<Double> position = motor.getRotorPosition();
    private StatusSignalValue<Double> velocity = motor.getRotorVelocity();
    private StatusSignalValue<Double> currentDraw = motor.getStatorCurrent();

    @Override
    public void setPosition(double degrees) {
        motor.setControl(motorRequest.withPosition((Units.degreesToRotations(degrees)*gearRatio)));    
    }

    @Override
    public void reset(double degrees){ 
        motor.setRotorPosition((Units.degreesToRotations(degrees))*gearRatio);
    }
   

    @Override
    public PivotIOInputsAutoLogged updateInputs() {
        // TODO Auto-generated method stub
        PivotIOInputsAutoLogged current = new PivotIOInputsAutoLogged();
        double supplyVoltage = supplyVoltageSignal.getValue();

        StatusSignalValue<Double> supplyVoltageSignal;
        // refresh the supply voltage signal
        supplyVoltageSignal.refresh();

        // wait up to 1 robot loop iteration (20ms) for fresh data
        supplyVoltageSignal.waitForUpdate(0.020);

        // disable supply voltage reporting (0 Hz)
        supplyVoltageSignal.setUpdateFrequency(0);
    }
}

//TODO
    // update inputs
        // stater current
