// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriverTrain extends SubsystemBase {
  private final SparkMax FRONT_LEFT_MOTOR; 
  private final SparkMax FRONT_RIGHT_MOTOR;
  private final SparkMax BACK_LEFT_MOTOR;
  private final SparkMax BACK_RIGHT_MOTOR;

  private final DifferentialDrive drive;

  /** Creates a new DriverTrain. */
  public DriverTrain() {
    FRONT_LEFT_MOTOR = new SparkMax(DriveConstants.FRONT_LEFT_MOTOR_PORT, MotorType.kBrushless);
    FRONT_RIGHT_MOTOR = new SparkMax(DriveConstants.FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushless);
    BACK_LEFT_MOTOR= new SparkMax(DriveConstants.BACK_LEFT_MOTOR_PORT, MotorType.kBrushless);
    BACK_RIGHT_MOTOR = new SparkMax(DriveConstants.BACK_RIGHT_MOTOR_PORT, MotorType.kBrushless);

    drive = new DifferentialDrive(FRONT_LEFT_MOTOR, FRONT_RIGHT_MOTOR);

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(60);

    config.follow(FRONT_LEFT_MOTOR);
    BACK_LEFT_MOTOR.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(FRONT_RIGHT_MOTOR);
    BACK_RIGHT_MOTOR.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.inverted(true);
    config.idleMode(IdleMode.kBrake);
    FRONT_LEFT_MOTOR.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }
}
