// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        public double velocityRPM;
        public double currentDrawAmps;
        public double temperatureCelsius;
        public double motorOutputVolts;
    }

    public abstract void in();
    public abstract void out();

    public abstract IntakeIOInputsAutoLogged updateInputs();
}

