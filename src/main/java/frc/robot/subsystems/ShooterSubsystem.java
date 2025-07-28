// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Kilo;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
// import frc.robot.Constants.ShooterConstants.ShooterAction;

public class ShooterSubsystem extends SubsystemBase {
  private SparkMax SHOOTER_LEFT_MOTOR = new SparkMax(ShooterConstants.SHOOTER_LEFT_MOTOR_PORT, MotorType.kBrushless);
  private SparkFlex SHOOTER_RIGHT_MOTOR = new SparkFlex(ShooterConstants.SHOOTER_RIGHT_MOTOR_PORT, MotorType.kBrushless);
  private SparkMaxConfig SHOOTER_LEFT_Config = new SparkMaxConfig();
  private SparkFlexConfig SHOOTER_RIGHT_Config = new SparkFlexConfig();
  private RelativeEncoder ShooterLEncoder = SHOOTER_LEFT_MOTOR.getEncoder();
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // SHOOTER_RIGHT_Config.follow(SHOOTER_LEFT_MOTOR, false);
   
    SHOOTER_RIGHT_Config
    .idleMode(IdleMode.kCoast)
    .inverted(false)
    .smartCurrentLimit(80);

    SHOOTER_LEFT_Config
    .idleMode(IdleMode.kCoast)
    .inverted(false)
    .smartCurrentLimit(80);

    SHOOTER_RIGHT_MOTOR.configure(SHOOTER_RIGHT_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SHOOTER_LEFT_MOTOR.configure(SHOOTER_LEFT_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getVelocity() {
    return ShooterLEncoder.getVelocity();
  }

  public void setShooterAction() {
    SHOOTER_LEFT_MOTOR.set(ShooterConstants.kLShoote);
    SHOOTER_RIGHT_MOTOR.set(ShooterConstants.kRShoote);
  }

  public void StopMotor() {
    SHOOTER_LEFT_MOTOR.set(ShooterConstants.kStop);
    SHOOTER_RIGHT_MOTOR.set(ShooterConstants.kStop);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Velocity", getVelocity());
  }
}
