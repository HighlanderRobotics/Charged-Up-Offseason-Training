// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DrivebaseSubsystem;

import java.sql.Driver;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems.DrivebaseSubsystem.SwerveModuleIO.SwerveModuleIOInputs;

/** Add your docs here. */
public class SwerveSubsystem extends SubsystemBase {


    SwerveModuleIOSim frontLeftIo;
    SwerveModuleIOSim frontRightIo;
    SwerveModuleIOSim backLeftIo;
    SwerveModuleIOSim backRightIo;

    SwerveModuleIOInputsAutoLogged frontLeftInputs;
    SwerveModuleIOInputsAutoLogged frontRightInputs;
    SwerveModuleIOInputsAutoLogged backLeftInputs;
    SwerveModuleIOInputsAutoLogged backRightInputs;

    double heading = 0.0;

    

    // Creating kinematics object using the module locations
    // Copied from documentation
    SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        Constants.FRONT_LEFT_LOCATION, 
        Constants.FRONT_RIGHT_LOCATION, 
        Constants.BACK_LEFT_LOCATION,
        Constants.BACK_RIGHT_LOCATION
    );

    

    Pose2d pose = new Pose2d(5.0, 13.5, new Rotation2d());
    
    ChassisSpeeds speeds = new ChassisSpeeds();
    SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(speeds);

    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];

    

    

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            swerveKinematics, new Rotation2d(),
        new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        }, pose);
    
    public SwerveSubsystem(SwerveModuleIOSim frontLeftIo, SwerveModuleIOSim frontRightIo, SwerveModuleIOSim backLeftIo,SwerveModuleIOSim backRightIo){
        this.frontLeftIo = frontLeftIo;
        this.frontRightIo = frontRightIo;
        this.backLeftIo = backLeftIo;
        this.backRightIo = backRightIo;
        frontLeftInputs = new SwerveModuleIOInputsAutoLogged();
        frontRightInputs = new SwerveModuleIOInputsAutoLogged();
        backLeftInputs = new SwerveModuleIOInputsAutoLogged();
        backRightInputs = new SwerveModuleIOInputsAutoLogged();
    }

    public CommandBase drive(DoubleSupplier forward, DoubleSupplier side, DoubleSupplier theta, boolean isFieldRelative){
        return new RunCommand(() -> {
            speeds = new ChassisSpeeds(forward.getAsDouble(), side.getAsDouble(), theta.getAsDouble());
         if(isFieldRelative){
            
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, odometry.getPoseMeters().getRotation().plus(
                Rotation2d.fromDegrees(DriverStation.getAlliance() == Alliance.Red ? 180 : 0)
                ));
         }
         
         
         heading += theta.getAsDouble() * 0.02;

        // Convert to module states
        moduleStates = swerveKinematics.toSwerveModuleStates(speeds);
         frontLeft = moduleStates[0];
         frontRight = moduleStates[1];
         backLeft = moduleStates[2];
         backRight = moduleStates[3];


        

            frontLeftIo.setDrive(frontLeft.angle.getDegrees(), frontLeft.speedMetersPerSecond);
            frontRightIo.setDrive(frontRight.angle.getDegrees(), frontRight.speedMetersPerSecond);
            backLeftIo.setDrive(backLeft.angle.getDegrees(), backLeft.speedMetersPerSecond);
            backRightIo.setDrive(backRight.angle.getDegrees(), backRight.speedMetersPerSecond);

            

        }, this);
    }
 
    @Override
    public void periodic(){
        
        frontLeftInputs = frontLeftIo.updateInputs();
        frontRightInputs = frontRightIo.updateInputs();
        backLeftInputs = backLeftIo.updateInputs();
        backRightInputs = backRightIo.updateInputs();
        
        Logger.getInstance().processInputs("Front Left Swerve", frontLeftInputs);
        Logger.getInstance().processInputs("Front Right Swerve", frontRightInputs);
        Logger.getInstance().processInputs("Back Left Swerve", backLeftInputs);
        Logger.getInstance().processInputs("Back Right Swerve", backRightInputs);
        Logger.getInstance().recordOutput("Pose", pose);




        pose = odometry.update(Rotation2d.fromRadians(heading % 2* Math.PI),
        new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeftInputs.drivePositionMeters,frontLeft.angle),
            new SwerveModulePosition(frontRightInputs.drivePositionMeters,frontRight.angle),
            new SwerveModulePosition(backLeftInputs.drivePositionMeters,backLeft.angle),
            new SwerveModulePosition(backRightInputs.drivePositionMeters,backRight.angle)
    });

    Logger.getInstance()
        .recordOutput(
            "Swerve States",
            new double[] {
                frontLeftInputs.swerveRotationRadians,
                frontLeftInputs.drivePositionMeters,
                frontRightInputs.swerveRotationRadians,
                frontRightInputs.drivePositionMeters,
                backLeftInputs.swerveRotationRadians,
                backLeftInputs.drivePositionMeters,
                backRightInputs.swerveRotationRadians,
                backRightInputs.drivePositionMeters,
            });

    }
    
}
