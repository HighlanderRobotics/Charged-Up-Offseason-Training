// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DrivebaseSubsystem;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/** Does not work as real yet. */
public class SwerveModuleIOReal implements SwerveModuleIO{

    TalonFX swerveMotor;
    TalonFX driveMotor;

    StatusSignal<Double> swerveSignalVoltage;
    StatusSignal<Double> driveSignalVoltage;
    
    StatusSignal<Double> swerveSignalPosition;

    CANcoder encoder;

    VoltageOut swerveVoltage = new VoltageOut(0);
    VoltageOut driveVoltage = new VoltageOut(0);


    PositionVoltage swerveRequest = new PositionVoltage(0);
    VelocityVoltage driveRequest = new VelocityVoltage(0);

    double encoderOffset;
    
    public SwerveModuleIOReal(int swerveID, int driveID, int encoderID, double encoderOffset){
        swerveMotor = new TalonFX(swerveID);
        driveMotor = new TalonFX(driveID);
        encoder = new CANcoder(encoderID);

        this.encoderOffset = encoderOffset;

        TalonFXConfiguration driveConfig  = new TalonFXConfiguration();
        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        encoderConfig.MagnetSensor.MagnetOffset = encoderOffset;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        
        turnConfig.ClosedLoopGeneral.ContinuousWrap = false;
        turnConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turnConfig.Feedback.SensorToMechanismRatio = 1.0;
        turnConfig.Feedback.RotorToSensorRatio = Constants.ROTATION_GEAR_RATIO;
        

        turnConfig.Slot0.kP = 1.0;
        turnConfig.Slot0.kD = 0;
        turnConfig.Slot0.kI = 0;
        turnConfig.Slot0.kS = 0.2;

        driveConfig.Slot0.kP = 0.05;
        driveConfig.Slot0.kD = 0;
        driveConfig.Slot0.kI = 0;
        
        encoder.getConfigurator().apply(encoderConfig);
        
        driveMotor.getConfigurator().apply(driveConfig);
       // driveMotor.setNeutralMode(NeutralModeValue.Brake);
        swerveMotor.getConfigurator().apply(turnConfig);
        Timer.delay(0.1);
        resetEncoder();
    }

    @Override
    public SwerveModuleIOInputsAutoLogged updateInputs() {

        SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

        swerveSignalVoltage = swerveMotor.getSupplyVoltage();
        driveSignalVoltage = driveMotor.getSupplyVoltage();
        swerveSignalPosition = swerveMotor.getRotorPosition();

        inputs.swerveOutputVolts = swerveSignalVoltage.getValue();
        inputs.driveOutputVolts = driveSignalVoltage.getValue();

        inputs.encoderPosition = encoder.getAbsolutePosition().getValue();
    
        inputs.swerveVelocityMetersPerSecond = 0.0;
        inputs.driveVelocityMetersPerSecond = 0.0;
        
        inputs.swerveRotationRotations = swerveSignalPosition.getValue();
        inputs.drivePositionMeters = (driveMotor.getPosition().getValue() / 6.86 ) * (4 * Math.PI); // 6.86 to adjust for gear ratio, and then multiply by circumfrence

        return inputs;
       

        


    }

    

    @Override
    public void setDrive(Rotation2d rotation, double position) {
        
        swerveRequest.Slot = 0;
        driveRequest.Slot = 0;

        swerveMotor.setControl(swerveRequest.withPosition(rotation.getRotations() * Constants.ROTATION_GEAR_RATIO)); // adjust rotations for gear ratio
        driveMotor.setControl(driveRequest.withVelocity(position * Constants.DRIVE_GEAR_RATIO)); // adjust rotations for gear ratio

    }

    @Override
    public void setDriveVoltage(double rotation, double driveVoltage2) {
        swerveMotor.setControl(swerveRequest.withPosition(rotation * Constants.ROTATION_GEAR_RATIO)); // adjust rotations for gear ratio
        driveMotor.setControl(driveVoltage.withOutput(driveVoltage2 * Constants.DRIVE_GEAR_RATIO)); // adjust rotations for gear ratio
        
    }

    @Override
    public void resetEncoder(){
        
        swerveMotor.setRotorPosition((encoder.getAbsolutePosition().getValue()) * Constants.ROTATION_GEAR_RATIO);
        System.out.println("reset swerbe heading");
    }
}
