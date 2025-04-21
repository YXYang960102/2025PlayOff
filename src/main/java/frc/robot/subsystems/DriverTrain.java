// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriverTrain extends SubsystemBase {
  private SparkMax FRONT_LEFT_MOTOR = new SparkMax(DriveConstants.FRONT_LEFT_MOTOR_PORT, MotorType.kBrushless);
  private SparkMax FRONT_RIGHT_MOTOR = new SparkMax(DriveConstants.FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushless);
  private SparkMax BACK_LEFT_MOTOR = new SparkMax(DriveConstants.BACK_LEFT_MOTOR_PORT, MotorType.kBrushless);
  private SparkMax BACK_RIGHT_MOTOR = new SparkMax(DriveConstants.BACK_RIGHT_MOTOR_PORT, MotorType.kBrushless);

  private SparkMaxConfig FRONT_LEFT_CONFIG = new SparkMaxConfig();

  private DifferentialDrive m_drive = new DifferentialDrive(FRONT_LEFT_MOTOR, FRONT_RIGHT_MOTOR);

  /** Creates a new DriverTrain. */
  public DriverTrain() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
