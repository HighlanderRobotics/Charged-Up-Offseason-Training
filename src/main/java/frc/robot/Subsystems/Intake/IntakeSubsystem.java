// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class IntakeSubsystem extends SubsystemBase{
    IntakeIO io;
    IntakeIOInputsAutoLogged inputs;

    public IntakeSubsystem (IntakeIO io) {
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();
    }

    public CommandBase intake() {
        return new RunCommand(() -> {
            io.in();
        }, this);
    }

    public CommandBase outtake() {
        return new RunCommand(() -> {
            io.out();
        }, this);
    }

    @Override
    public void periodic() {
        inputs = io.updateInputs();
        Logger.getInstance().processInputs("Intake",inputs);
    }


}
