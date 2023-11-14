// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import com.ctre.phoenixpro.hardware.TalonFX;

/** Add your docs here. */
public class IntakeIOReal implements IntakeIO {

    private final TalonFX motor = new TalonFX(0);


    public IntakeIOInputsAutoLogged updateInputs() {

            IntakeIOInputsAutoLogged input = new IntakeIOInputsAutoLogged();

            input.velocityRPM = motor.getVelocity().getValue() * 60;
            input.currentDrawAmps = motor.getTorqueCurrent().getValue();
            input.temperatureCelsius = 0;
            input.motorOutputVolts = motor.getSupplyVoltage().getValue();

            return input;
        }
    
    @Override

    public void in() {
        motor.setVoltage(10);
    }

    public void out() {
        motor.setVoltage(-10);
    }

    

}
