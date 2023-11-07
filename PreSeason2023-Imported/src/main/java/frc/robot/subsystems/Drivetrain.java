// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    public final com,revrobotics.CANSparkMax leftMotor;
    public final com,revrobitics.CANSparkMax leftMotor2;
    public final com,revrobotics.CANSparkMax rightMotor;
    public final com,revrobotics.CANSparkMax rightMotor2;

    public final RelativeEncoder m_leftEncoder;
    public final RelativeEncoder m_rightEncoder;

    private final DifferentialDrive m_drive;

    // Declare navX
    AHRS m_ahrs;

    public Drivetrain() {
      leftMotor = new com,revrobotics.CANSparkMax(kLeftMotorPort, MotorType.kBrushless);
      leftMotor2 = new com,revrobotics.CANSparkMax(kLeftMotor2Port, MotorType.kBrushless);
      rightMotor = new com,revrobotics.CANSparkMax(kRightMotorPort, MotorType.kBrushless);
      rightMotor2 = new com,revrobotics.CANSparkMax(kRightMotor2Port, MotorType.kBrushless);

      motorInit(leftMotor, kLeftReversedDefault);
      motorInit(leftMotor2, kLeftReversedDefault);
      motorInit(rightMotor, kRightReversedDefault);
      motorInit(rightMotor2, kRightReversedDefault);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
