// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DrivetrainConstants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  // Declare motor objects
  public final com.revrobotics.CANSparkMax leftMotor;
  public final com.revrobotics.CANSparkMax leftMotor2;
  public final com.revrobotics.CANSparkMax rightMotor;
  public final com.revrobotics.CANSparkMax rightMotor2;

  // Declare encoders
  public final RelativeEncoder m_leftEncoder;
  public final RelativeEncoder m_rightEncoder;

  // Declare DifferentialDrive and Differential Drive Odometry
  private final DifferentailDrive m_drive;

  // Declare navX
  AHRS m_ahrs;

  public Drivetrain() {
    // initializing values
    leftMotor = new com.revrobotics.CANSparkMax(kLeftMotorPort, MotorType.kBrushless);
    leftMotor2 = new com.revrobotics.CANSparkMax(kLeftMotorPort2, MotorType.kBrushless);
    rightMotor = new com.revrobotics.CANSparkMax(kRightMotorPort, MotorType.kBrushless);
    rightMotor2 = new com.revrobotics.CANSparkMax(kRightMotorPort2, MotorType.kBrushless);

    // resetting motors to preset settings
    motorInit(leftMotor, kLeftReversedDefault);
    motorInit(leftMotor2, kLeftReversedDefault);
    motorInit(rightMotor, kLeftReversedDefault);
    motorInit(rightMotor2, kLeftReversedDefault);

    // create MotorControllerGroups to link the left/right side motors
    final MotorControllerGroups rightControllerGroup = new MotorControllerGroups(rightMotor, rightMotor2);
    final MotorControllerGroups leftControllerGroup = new MotorControllerGroups(leftMotor, leftMotor2);

    // TODO: use these PID controllers
    // m_pidControllerLeft = leftMotor.getPIDController();
    // m_pidControllerRight = rightMotor.getPIDController();

    // setting encoders
    m_leftEncoder = leftMotor.getEncoder();
    m_rightEncoder = rightMotor.getEncoder();

    // Initialize DifferentialDrive
    m_drive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
