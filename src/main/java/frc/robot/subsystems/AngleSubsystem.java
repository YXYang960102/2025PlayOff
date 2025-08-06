// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.AngleAction;
import frc.robot.Constants.IntakeConstants.IntakeState;

public class AngleSubsystem extends SubsystemBase {
  private SparkMax Angle_MOTOR = new SparkMax(IntakeConstants.ANGLE_MOTOR_PORT, MotorType.kBrushless);
  private SparkMaxConfig AngleConfig = new SparkMaxConfig();
  private RelativeEncoder AngleEncoder =  Angle_MOTOR.getEncoder();
  private SparkAbsoluteEncoder AngleAbsEncoder = Angle_MOTOR.getAbsoluteEncoder();
  private SparkClosedLoopController AnglePIDController = Angle_MOTOR.getClosedLoopController();
  /** Creates a new AngleSubsystem. */
  public AngleSubsystem() {
    
    AngleEncoder.setPosition(AngleAbsEncoder.getPosition());

    AngleConfig.softLimit
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimitEnabled(true)
    .forwardSoftLimit(IntakeConstants.kAngleUpLimit)
    .reverseSoftLimit(IntakeConstants.kAngleDownLimit);

    AngleConfig
    .inverted(true)
    .idleMode(IdleMode.kBrake)
    .closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .pid(IntakeConstants.AnglekP, IntakeConstants.AnglekI, IntakeConstants.AnglekD)
    .iZone(IntakeConstants.AnglekIz)
    .velocityFF(IntakeConstants.AnglekFF)
    .maxOutput(IntakeConstants.AnglekMaxOutput)
    .minOutput(IntakeConstants.AnglekMinOutput);

    Angle_MOTOR.configure(AngleConfig, ResetMode.kNoResetSafeParameters,
     PersistMode.kPersistParameters);

  }

  public double getAnglePosition() {
    return AngleEncoder.getPosition();
  }

  public double getAbsPosition() {
    return AngleAbsEncoder.getPosition();
  }

  public void setAngleAction(AngleAction action) {
    Angle_MOTOR.set(action.rate);
  }

  public void setState(IntakeState state) {
    AnglePIDController.setReference(state.position, ControlType.kPosition);
  }

  public void setAngleHold() {
    AnglePIDController.setReference(getAbsPosition(), ControlType.kPosition);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Angle Position", getAnglePosition());
    SmartDashboard.putNumber("Angle Abs Position", getAbsPosition());
  }
}
