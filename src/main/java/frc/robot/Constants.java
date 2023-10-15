// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static final double DRIVEBASE_MAX_SPEED_FPS = Units.feetToMeters(16);

    public static final int MOTOR_ID_FRONT_LEFT_DRIVE = 0;
    public static final int MOTOR_ID_FRONT_LEFT_ROTATION = 1;

    public static final int MOTOR_ID_FRONT_RIGHT_DRIVE = 2;
    public static final int MOTOR_ID_FRONT_RIGHT_ROTATION = 3;

    public static final int MOTOR_ID_BACK_LEFT_DRIVE = 4;
    public static final int MOTOR_ID_BACK_LEFT_ROTATION = 5;

    public static final int MOTOR_ID_BACK_RIGHT_DRIVE = 6;
    public static final int MOTOR_ID_BACK_RIGHT_ROTATION = 7;

    public static final double ROTATION_GEAR_RATIO = 12.8;
    public static final double DRIVE_GEAR_RATIO = 6.86;

    public static final int GYRO_MODULE_ID = 8;

    // Locations for the swerve drive modules relative to the robot center.
    // Copied from documentation
    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(0.381, 0.381);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(0.381, -0.381);
    public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-0.381, 0.381);
    public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-0.381, -0.381);

}
