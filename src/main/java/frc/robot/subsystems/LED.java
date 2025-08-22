// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.LEDConstants;

import java.util.Map;

public class LED extends SubsystemBase {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private LEDPattern m_currentPattern = LEDPattern.solid(Color.kBlack); // 預設關燈

  private boolean intakeActive = false;
  private boolean hasBall = false;
  private boolean shooterActive = false;
  private boolean shooterReverseActive = false;
  private boolean shooterRobotBreaker = false;

  private static final LEDPattern kYellowBlink = LEDPattern.solid(Color.kYellow).blink(
      Units.Seconds.of(0.2),
      Units.Seconds.of(0.2));
  private static final LEDPattern kRedBlink = LEDPattern.solid(Color.kRed).blink(
      Units.Seconds.of(0.2),
      Units.Seconds.of(0.2));
  private static final LEDPattern kPinkBlink = LEDPattern.solid(Color.kPink).blink(
      Units.Seconds.of(0.2),
      Units.Seconds.of(0.2));

  private static final LEDPattern kIdlePattern = LEDPattern.rainbow(255, 255).breathe(Units.Seconds.of(2.0));

  /** Creates a new LED. */
  public LED(int pwmPort, int length) {
    m_led = new AddressableLED(pwmPort);
    m_ledBuffer = new AddressableLEDBuffer(length);
    m_led.setLength(length);
    m_led.start();
  }

  public void setIntakeActive(boolean active) {
    intakeActive = active;
  }

  public void setHasBall(boolean detected) {
    hasBall = detected;
  }

  public void setShooterActive(boolean active) {
    shooterActive = active;
  }

  public void setShooterReverseActive(boolean active) {
    shooterReverseActive = active;
  }

  public void setShooterBreaker(boolean active) {
    shooterRobotBreaker = active;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (shooterActive) {
      LEDPattern base = LEDPattern.gradient(GradientType.kContinuous ,Color.kFloralWhite, Color.kBlack);

// 每秒跑 50 cm，大概 59 顆 LED
Distance ledSpacing = Units.Centimeters.of(100.0 / 53.0);

m_currentPattern = base.scrollAtAbsoluteSpeed(
    Units.Centimeters.per(Units.Second).of(50), // 調整速度
    ledSpacing
);

    } else if (shooterReverseActive) {
      // 逆發射 → 閃爍黃燈
      m_currentPattern = kYellowBlink;

    } else if (hasBall) {
      // 球 → 恆亮綠燈
      m_currentPattern = LEDPattern.solid(Color.kGreen);

    } else if (intakeActive) {
      // Intake → 閃爍紅燈
      m_currentPattern = kRedBlink;

    } else if(shooterRobotBreaker) {
      m_currentPattern = kPinkBlink;
    } else {
      // 平時 → 燈條關閉
      m_currentPattern = kIdlePattern;
    }

    // 套用 pattern
    m_currentPattern.applyTo(m_ledBuffer);
    m_led.setData(m_ledBuffer);
  }
}
