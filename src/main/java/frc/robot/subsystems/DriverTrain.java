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
  private  SparkMax FRONT_LEFT_MOTOR = new SparkMax(DriveConstants.FRONT_LEFT_MOTOR_PORT, MotorType.kBrushless); 
  private  SparkMax FRONT_RIGHT_MOTOR = new SparkMax(DriveConstants.FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushless);
  private  SparkMax BACK_LEFT_MOTOR = new SparkMax(DriveConstants.BACK_LEFT_MOTOR_PORT, MotorType.kBrushless);
  private  SparkMax BACK_RIGHT_MOTOR = new SparkMax(DriveConstants.BACK_RIGHT_MOTOR_PORT, MotorType.kBrushless);

  private SparkMaxConfig FRONT_LEFT_CONFIG = new SparkMaxConfig();
  private SparkMaxConfig FRONT_RIGHT_CONFIG = new SparkMaxConfig();
  private SparkMaxConfig BACK_LEFT_CONFIG = new SparkMaxConfig();
  private SparkMaxConfig BACK_RIGHT_CONFIG = new SparkMaxConfig();

  private  DifferentialDrive drive = new DifferentialDrive(FRONT_LEFT_MOTOR, FRONT_RIGHT_MOTOR);;


  /** Creates a new DriverTrain. */
  public DriverTrain() {

    FRONT_LEFT_CONFIG
    .inverted(true)
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(60);

    FRONT_RIGHT_CONFIG
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(60);

    BACK_LEFT_CONFIG.follow(FRONT_LEFT_MOTOR);
    BACK_RIGHT_CONFIG.follow(FRONT_RIGHT_MOTOR);

    FRONT_LEFT_MOTOR.configure(FRONT_LEFT_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    FRONT_RIGHT_MOTOR.configure(FRONT_RIGHT_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    BACK_LEFT_MOTOR.configure(BACK_LEFT_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    BACK_RIGHT_MOTOR.configure(BACK_RIGHT_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    drive.setDeadband(0.05);
    // SparkMaxConfig config = new SparkMaxConfig();
    // config.smartCurrentLimit(60);

    // config.follow(FRONT_LEFT_MOTOR);
    // BACK_LEFT_MOTOR.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // config.follow(FRONT_RIGHT_MOTOR);
    // BACK_RIGHT_MOTOR.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // config.inverted(true);
    // config.idleMode(IdleMode.kBrake);
    // FRONT_LEFT_MOTOR.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }
}
