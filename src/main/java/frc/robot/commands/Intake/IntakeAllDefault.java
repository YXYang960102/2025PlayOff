// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants.IntakeAction;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAllDefault extends InstantCommand {
  private IntakeSubsystem intakeSubsystem;
  private AngleSubsystem angleSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private LED led;
  public IntakeAllDefault(
    IntakeSubsystem intakeSubsystem,
    AngleSubsystem angleSubsystem,
    ShooterSubsystem shooterSubsystem,
    LED led
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.angleSubsystem = angleSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.led = led;

    addRequirements(intakeSubsystem, angleSubsystem, shooterSubsystem, led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setIntakeAction(IntakeAction.kStop);
    angleSubsystem.setState(IntakeState.kDefult);
    shooterSubsystem.StopMotor();
    led.setIntakeActive(false);
  }
}
