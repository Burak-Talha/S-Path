// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DRIVE_CONSTANTS{
        public static final int FRONT_LEFT_MOTOR_PIN = 0;
        public static final int FRONT_RIGHT_MOTOR_PIN = 1;
        public static final int REAR_LEFT_MOTOR_PIN = 2;
        public static final int REAR_RIGHT_MOTOR_PIN = 3;

        public static final int LEFT_DRIVE_ENCODER_A_CHANNEL = 9;
        public static final int LEFT_DRIVE_ENCODER_B_CHANNEL = 8;
        public static final int RIGHT_DRIVE_ENCODER_A_CHANNEL = 0;
        public static final int RIGHT_DRIVE_ENCODER_B_CHANNEL = 1;

        public static final double TANK_DRIVE_LEFT_SPEED = 0.85;
        public static final double TANK_DRIVE_RIGHT_SPEED = 0.85;
        
        public static final double ARCADE_DRIVE_X_SPEED = 0.7;
        public static final double ARCADE_DRIVE_Z_SPEED = 0.7;  
        
        public static final double K_DRIVE_TICK_2_FEET_1 = 1.0 / 128 * 6 * Math.PI / 12;

        public static final double KP = 1;
        public static final double GYRO_KP = 1;
        public static final double KI = 0.6;
        public static final double KD = 0.49;

        public static final double kTrackwidthMeters = 0.69;

        public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

        // For Simple Motor Feedforward
        public static final double KS_VOLTS = 5.0;
        public static final double KV_VOLTS = 5.0;

        public static double kMaxSpeedMetersPerSecond = 10;
        public static double kMaxAccelerationMetersPerSecondSquared = 12;
    }
}
