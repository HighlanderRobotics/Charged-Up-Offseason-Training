// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DrivebaseSubsystem;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveSubsystem extends SubsystemBase {


    SwerveModuleIO frontLeftIo;
    SwerveModuleIO frontRightIo;
    SwerveModuleIO backLeftIo;
    SwerveModuleIO backRightIo;

    SwerveModuleIOInputsAutoLogged frontLeftInputs;
    SwerveModuleIOInputsAutoLogged frontRightInputs;
    SwerveModuleIOInputsAutoLogged backLeftInputs;
    SwerveModuleIOInputsAutoLogged backRightInputs;

    GyroModuleIOInputsAutoLogged gyroInputs;

    GyroModuleIOReal gyro;

    double heading = 0.0;

    CANcoder encoder;

    
    

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
    



    public SwerveSubsystem(SwerveModuleIO frontLeftIo, SwerveModuleIO frontRightIo, SwerveModuleIO backLeftIo,SwerveModuleIO backRightIo, GyroModuleIOReal gyroIo){

        this.frontLeftIo = frontLeftIo;
        this.frontRightIo = frontRightIo;
        this.backLeftIo = backLeftIo;
        this.backRightIo = backRightIo;
        this.gyro = gyroIo;
        frontLeftInputs = new SwerveModuleIOInputsAutoLogged();
        frontRightInputs = new SwerveModuleIOInputsAutoLogged();
        backLeftInputs = new SwerveModuleIOInputsAutoLogged();
        backRightInputs = new SwerveModuleIOInputsAutoLogged();
        gyroInputs = new GyroModuleIOInputsAutoLogged();

        SmartDashboard.putData("drive zero", drive(() -> 0.0, () -> 0.0, () -> 0.0, false));
        SmartDashboard.putData("drive forward 0.5", drive(() -> 0.5, () -> 0.0, () -> 0.0, false));
        SmartDashboard.putData("Reseet Encoder", resetEncoder());
    }

    public CommandBase resetEncoder(){
        return new InstantCommand( () -> {
            frontLeftIo.resetEncoder();
            frontRightIo.resetEncoder();
            backLeftIo.resetEncoder();
            backRightIo.resetEncoder();
        },this);
        
    }

    public CommandBase drive(DoubleSupplier forward, DoubleSupplier side, DoubleSupplier theta, boolean isFieldRelative){
        return this.run(() -> {
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


        

            frontLeftIo.setDrive(frontLeft.angle, frontLeft.speedMetersPerSecond); 
            frontRightIo.setDrive(frontRight.angle, frontRight.speedMetersPerSecond); 
            backLeftIo.setDrive(backLeft.angle, backLeft.speedMetersPerSecond); 
            backRightIo.setDrive(backRight.angle, backRight.speedMetersPerSecond); 
            
            

        });
    }


    
    public CommandBase balance(){
        /*
         * Set the chassis motor controllers to brake mode. This didn’t have much effect on the performance of our routine (since we used sysid on our drivetrain and controlled via speed, not percent power), but it helped quell questions from alliance partners.
        Drive forward at 0.2m/s until we are at 10 degrees (or more) of pitch.
        Drive forward 0.65m at 0.3m/s. This just helped us get close to the center faster.
        Initialize bang-bang (ish) auto balancing:
        a. 
        b. 
        c. 
        Stop for 3 seconds. This prevents any issues with joystick drift causing the default drive command to move the robot once it is balanced. We don’t have issues with teleop being stuck due to teleop automatically cancelling all auto commands.
         */
        return Commands.either(
            drive(() -> 0,() -> 0,() -> 0, true), 
            // If the angle is less than 3 degrees and the charge station is not moving, don't move.
            Commands.either(
                drive(() -> 0,() -> 0,() -> 0,true),
                // If the charge station is currently pitching, stop thr drivetrain.
                  Commands.either(
                    drive(() -> -0.25, () -> 0, () -> 0, false).until(() -> (gyroInputs.pitch > 2 || gyroInputs.pitch < -2)),
                    drive(() -> 0.25, () -> 0, () -> 0, false).until(() -> (gyroInputs.pitch > 2 || gyroInputs.pitch < -2))
                    // If the pitch is positive, drive forwards.
                    // If the pitch is negative, drive backwards.
                    , () -> (gyroInputs.pitch > 0)), 
                  () -> (gyroInputs.angularRate > 0.05 || gyroInputs.angularRate < -0.05)), 
            () -> ((Math.abs(gyroInputs.pitch) < 4)));
    }
 
    @Override
    public void periodic(){
        
        frontLeftInputs = frontLeftIo.updateInputs();
        frontRightInputs = frontRightIo.updateInputs();
        backLeftInputs = backLeftIo.updateInputs();
        backRightInputs = backRightIo.updateInputs();
        gyroInputs = gyro.updateInputs();
        
        Logger.getInstance().processInputs("Front Left Swerve", frontLeftInputs);
        Logger.getInstance().processInputs("Front Right Swerve", frontRightInputs);
        Logger.getInstance().processInputs("Back Left Swerve", backLeftInputs);
        Logger.getInstance().processInputs("Back Right Swerve", backRightInputs);
        Logger.getInstance().processInputs("Gyro", gyroInputs);
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
