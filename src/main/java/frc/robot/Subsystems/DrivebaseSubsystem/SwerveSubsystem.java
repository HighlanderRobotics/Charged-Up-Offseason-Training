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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Subsystems.DrivebaseSubsystem.SwerveModuleIO.SwerveModuleIOInputs;

/** Add your docs here. */
public class SwerveSubsystem extends SubsystemBase {

    CommandXboxController controller = new CommandXboxController(0);

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

    public CommandBase drive(DoubleSupplier forward, DoubleSupplier side, DoubleSupplier omega, boolean isFieldRelative){
        
        return this.run(() -> {

            Logger.getInstance().recordOutput("forward", forward.getAsDouble());
            Logger.getInstance().recordOutput("side", side.getAsDouble());
            Logger.getInstance().recordOutput("theta", omega.getAsDouble());

            speeds = new ChassisSpeeds(forward.getAsDouble(), side.getAsDouble(), omega.getAsDouble());
         if(isFieldRelative){
            
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, odometry.getPoseMeters().getRotation().plus(
                Rotation2d.fromDegrees(DriverStation.getAlliance() == Alliance.Red ? 180 : 0)
                ));
         }
         
         
         heading += omega.getAsDouble() * 0.02;

        // Convert to module states
        moduleStates = swerveKinematics.toSwerveModuleStates(speeds);
        frontLeft = moduleStates[0];
        frontRight = moduleStates[1];
        backLeft = moduleStates[2];
        backRight = moduleStates[3];
        
        var frontLeftOptimize = SwerveModuleState.optimize(frontLeft, Rotation2d.fromRotations(frontLeftInputs.encoderPosition - Math.floor(frontLeftInputs.encoderPosition)));
        var frontRightOptimize = SwerveModuleState.optimize(frontRight, Rotation2d.fromRotations(frontRightInputs.encoderPosition - Math.floor(frontRightInputs.encoderPosition)));
        var backLeftOptimize = SwerveModuleState.optimize(backLeft, Rotation2d.fromRotations(backLeftInputs.encoderPosition - Math.floor(backLeftInputs.encoderPosition)));
        var backRightOptimize = SwerveModuleState.optimize(backRight, Rotation2d.fromRotations(backRightInputs.encoderPosition - Math.floor(backRightInputs.encoderPosition)));
        

        

            frontLeftIo.setDrive(frontLeftOptimize.angle, frontLeftOptimize.speedMetersPerSecond); 
            frontRightIo.setDrive(frontRightOptimize.angle, frontRightOptimize.speedMetersPerSecond); 
            backLeftIo.setDrive(backLeftOptimize.angle, backLeftOptimize.speedMetersPerSecond); 
            backRightIo.setDrive(backRightOptimize.angle, backRightOptimize.speedMetersPerSecond);
            Logger.getInstance().recordOutput("module states", moduleStates);
            

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
                Units.rotationsToRadians(frontLeftInputs.swerveRotationRotations) / Constants.ROTATION_GEAR_RATIO,
                frontLeftInputs.driveVelocityMetersPerSecond,
                Units.rotationsToRadians(frontRightInputs.swerveRotationRotations) / Constants.ROTATION_GEAR_RATIO,
                frontRightInputs.driveVelocityMetersPerSecond,
                Units.rotationsToRadians(backLeftInputs.swerveRotationRotations) / Constants.ROTATION_GEAR_RATIO,
                backLeftInputs.driveVelocityMetersPerSecond,
                Units.rotationsToRadians(backRightInputs.swerveRotationRotations) / Constants.ROTATION_GEAR_RATIO,
                backRightInputs.driveVelocityMetersPerSecond,
                
            });
            Logger.getInstance().recordOutput("Front Left Encoder Position", frontLeftInputs.encoderPosition * Constants.ROTATION_GEAR_RATIO);
            Logger.getInstance().recordOutput("Front Right Encoder Position", frontRightInputs.encoderPosition * Constants.ROTATION_GEAR_RATIO);
            Logger.getInstance().recordOutput("Back Left Encoder Position", backLeftInputs.encoderPosition * Constants.ROTATION_GEAR_RATIO);
            Logger.getInstance().recordOutput("Back Right Encoder Position", backRightInputs.encoderPosition * Constants.ROTATION_GEAR_RATIO);
    }
    
}
