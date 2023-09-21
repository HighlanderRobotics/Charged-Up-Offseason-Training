// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class PivotSubsystem extends SubsystemBase {
    PivotIO io;
    PivotIOInputsAutoLogged inputs;

    public PivotSubsystem(PivotIO io) {
        this.io = io;
        inputs =  new PivotIOInputsAutoLogged();
    }

    public CommandBase run(double degrees) {
        return new RunCommand(() -> {
            io.setPosition(degrees);
        }, this);
    }   

    @Override
    public void periodic() {
        inputs = io.updateInputs();
        Logger.getInstance().processInputs("Intake", inputs);
    }



}
