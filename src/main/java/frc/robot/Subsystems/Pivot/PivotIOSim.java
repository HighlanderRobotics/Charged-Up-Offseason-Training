// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Pivot;

import com.ctre.phoenixpro.hardware.TalonFX;

/** Add your docs here. */
public class PivotIOSim implements PivotIO{
    private final TalonFX motor = new TalonFX(0);

    public PivotIOInputsAutoLogged updateInputs() {

        PivotIOInputsAutoLogged input = new PivotIOInputsAutoLogged();

        //var motorSimState = motor.getSimState();

        input.velocityRPM = motor.getVelocity().getValue()*60;
        input.currentDrawAmps = motor.getStatorCurrent().getValue();
        input.temperatureCelsius = 0;
        input.motorOutputVolts = motor.getSupplyVoltage().getValue();
        input.positionDegrees = motor.getPosition().getValue();

        return input;

    }

    @Override
    public void setPosition(double degrees) {
        //Set motor position
    }
    
}
