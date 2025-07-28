// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.AngleAction;
import frc.robot.subsystems.AngleSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeStateNormal extends Command {
  private AngleSubsystem AngleSubsystem;
  private AngleAction angleAction;
  /** Creates a new IntakeStateNormal. */
  public IntakeStateNormal(
    AngleSubsystem AngleSubsystem,
    AngleAction angleAction
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.AngleSubsystem = AngleSubsystem;
    this.angleAction = angleAction;
    addRequirements(AngleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AngleSubsystem.setAngleAction(angleAction);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    AngleSubsystem.setAngleAction(AngleAction.kStop);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
