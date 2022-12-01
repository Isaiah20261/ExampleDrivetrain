// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class DrivetrainConstants {
        public static final double kControllerDeadzone = 0.05;

        // based off PDP ports and the REV Spark Client. 0 is reassigned as 15.
        public static final int kLeftMotorPort = 12;
        public static final int kRightMotorPort = 15;
        public static final int kLeftMotor2Port = 14;
        public static final int kRightMotor2Port = 1;

        // since the encoder is build into the motor we need to account for gearing
        public static final double kWheelDiameter = 6.0;
        public static final double kGearRatio = 1.0 / 12.0;
        public static final double kDistancePerRevolution = kWheelDiameter * kGearRatio * 3.14;
        public static final double kSpeedPerRevolution = kDistancePerRevolution / 60.0;

        public static final int kCurrentLimit = 60;
        public static final boolean kLeftReversedDefault = true;
        public static final boolean kRightReversedDefault = !kLeftReversedDefault;

        public static final int kStallLimit = 45;
        public static final double kTurnAngleD = 0.0;
        public static final double kTurnAngleI = 0.0;
        public static final double kTurnAngleP = 0.01;
        public static final double kTurnAngleTolerace = 8.0;
        public static final double kTurnSpeedTolerance = 5.0;
        public static final double kAutoForwardI = 0.0001;
        public static final double kAutoForwardP = 0.009;
        public static final double kAutoForwardD = 0.00;
        public static final double kVelocityTolerance = 0.2;
        public static final double kPositionTolerace = 5.0;
    }

}