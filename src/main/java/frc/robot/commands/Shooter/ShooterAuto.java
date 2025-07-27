// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakeAction;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterAuto extends Command {
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private Timer timer = new Timer();
  /** Creates a new ShooterAuto. */
  public ShooterAuto(
    ShooterSubsystem shooterSubsystem,
    IntakeSubsystem intakeSubsystem
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(shooterSubsystem, intakeSubsystem);
    timer.reset();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    shooterSubsystem.setShooterAction();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.hasElapsed(1.0)) { // pass 1 sencond
      intakeSubsystem.setIntakeAction(IntakeAction.kShoote); // start intake
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.StopMotor();
    intakeSubsystem.setIntakeAction(IntakeAction.kStop);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(3.0);
  }
}
