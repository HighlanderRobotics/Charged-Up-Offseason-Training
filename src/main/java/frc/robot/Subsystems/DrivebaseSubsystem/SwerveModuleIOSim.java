// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DrivebaseSubsystem;

import com.ctre.phoenixpro.controls.ControlRequest;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/** Add your docs here. */
public class SwerveModuleIOSim implements SwerveModuleIO{

    TalonFX swerveMotor;
    TalonFX driveMotor;

    VoltageOut swerveVoltage = new VoltageOut(0);
    VoltageOut driveVoltage = new VoltageOut(0);


    PositionVoltage swerveRequest = new PositionVoltage(0);
    VelocityVoltage driveRequest = new VelocityVoltage(0);

    public SwerveModuleIOSim(int swerveID, int driveID){
        swerveMotor = new TalonFX(swerveID);
        driveMotor = new TalonFX(driveID);
    }

    @Override
    public SwerveModuleIOInputsAutoLogged updateInputs() {

        SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
        var swerveSimState = swerveMotor.getSimState();
        var driveSimState = driveMotor.getSimState();
        swerveSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());
        driveSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());
        inputs.swerveOutputVolts = swerveSimState.getMotorVoltage();
        inputs.driveOutputVolts = driveSimState.getMotorVoltage();
    
        inputs.swerveVelocityMetersPerSecond = 0.0;
        inputs.driveVelocityMetersPerSecond = 0.0;
    
        inputs.swervePositionMeters = 0.0;
        inputs.drivePositionMeters = (driveMotor.getPosition().getValue() / 6.86 ) * (4 * Math.PI); // 6.86 to adjust for gear ratio, and then multiply by circumfrence
        
        inputs.swerveCurrentAmps = new double[] {swerveSimState.getTorqueCurrent()};
        inputs.swerveTempCelsius = new double[0];
        inputs.driveCurrentAmps = new double[] {driveSimState.getTorqueCurrent()};
        inputs.driveTempCelsius = new double[0];

        return inputs;
       

        


    }

    

    @Override
    public void setDrive(double rotation, double position) {
        

        swerveMotor.setControl(swerveRequest.withPosition(rotation * 12.8)); // adjust rotations for gear ratio
        driveMotor.setControl(driveRequest.withVelocity(position * 6.86)); // adjust rotations for gear ratio

    }
}
