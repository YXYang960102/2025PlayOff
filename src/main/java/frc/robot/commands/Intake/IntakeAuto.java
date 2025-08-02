// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakeAction;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAuto extends Command {
  private IntakeSubsystem intakeSubsystem;
  private IntakeAction intakeAction;
  private AngleSubsystem angleSubsystem;
  private Timer timer = new Timer();
  private boolean ballDetected = false;
  /** Creates a new IntakeAuto. */
  public IntakeAuto(
    IntakeSubsystem intakeSubsystem,
    IntakeAction intakeAction,
    AngleSubsystem angleSubsystem
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.intakeAction = intakeAction;
    this.angleSubsystem = angleSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setIntakeAction(intakeAction);
    angleSubsystem.setState(IntakeState.kGetBall);
    timer.reset();
    timer.stop();
    ballDetected = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!ballDetected && intakeSubsystem.getBall()) {
      ballDetected = true;
      timer.reset();
      timer.start();
    }

    if (ballDetected && timer.hasElapsed(0.05)) {
      intakeSubsystem.setIntakeAction(IntakeAction.kStop);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeAction(IntakeAction.kStop);
    angleSubsystem.setState(IntakeState.kDefult);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ballDetected && timer.hasElapsed(0.05);
  }
}
