// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.IntakeConstants.AngleAction;
import frc.robot.Constants.IntakeConstants.IntakeAction;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkMax Intake_MOTOR = new SparkMax(IntakeConstants.Intake_MOTOR_PORT, MotorType.kBrushless);
  // private SparkMax Angle_MOTOR = new SparkMax(IntakeConstants.ANGLE_MOTOR_PORT, MotorType.kBrushless);
  private SparkMaxConfig IntakeConfig = new SparkMaxConfig();
  // private SparkMaxConfig AngleConfig = new SparkMaxConfig();
  // private RelativeEncoder AngleEncoder =  Angle_MOTOR.getEncoder();
  // private SparkAbsoluteEncoder AngleAbsEncoder = Angle_MOTOR.getAbsoluteEncoder();
  // private SparkClosedLoopController AnglePIDController = Angle_MOTOR.getClosedLoopController();

  private I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 colorSensorV3  =new ColorSensorV3(i2cPort);

  public IntakeSubsystem() {
    // AngleEncoder.setPosition(AngleAbsEncoder.getPosition());
    colorSensorV3.configureProximitySensor(ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate12ms);

    IntakeConfig
    .inverted(true)
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(70);

    // AngleConfig.softLimit
    // .forwardSoftLimitEnabled(false)
    // .reverseSoftLimitEnabled(false)
    // .forwardSoftLimit(0)
    // .reverseSoftLimit(0);

    // AngleConfig
    // .inverted(false)
    // .idleMode(IdleMode.kBrake)
    // .closedLoop
    // .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    // .pid(0, 0, 0)
    // .iZone(0)
    // .velocityFF(0)
    // .maxOutput(0)
    // .minOutput(0);
    

    Intake_MOTOR.configure(IntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Angle_MOTOR.configure(AngleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  // public double getAnglePosition() {
  //   return AngleEncoder.getPosition();
  // }

  // public double getAbsPosition() {
  //   return AngleAbsEncoder.getPosition();
  // }

  public void setIntakeAction(IntakeAction action) {
   Intake_MOTOR.set(action.rate);
  }

  // public void setAngleAction(AngleAction action) {
  //   Angle_MOTOR.set(action.rate);
  // }

  // public void setState(IntakeState state) {
  //   AnglePIDController.setReference(state.position, ControlType.kPosition);
  // }

  // public void setAngleHold() {
  //   AnglePIDController.setReference(getAbsPosition(), ControlType.kPosition);
  // }

  public double getIR() {
  /**
  * The sensor returns a raw IR value of the infrared light detected.
  */
  double IR = colorSensorV3.getIR();
  return IR;
  }

  public int getProximity() {
  /**
  * In addition to RGB IR values, the color sensor can also return an
  * infrared proximity value. The chip contains an IR led which will emit
  * IR pulses and measure the intensity of the return. When an object is
  * close the value of the proximity will be large (max 2047 with default
  * settings) and will approach zero when the object is far away.
  *
  * Proximity can be used to roughly approximate the distance of an object
  * or provide a threshold for when an object is close enough to provide
  * accurate color values.
  */
  int proximity = colorSensorV3.getProximity();
  return proximity;
  }

  public boolean getBall() {
  return getProximity() > IntakeConstants.kcorlorSensorLGateValue;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("isGet", getBall());
    // SmartDashboard.putNumber("Angle Position", getAnglePosition());
    // SmartDashboard.putNumber("Angle Abs Position", getAbsPosition());
  }
}
