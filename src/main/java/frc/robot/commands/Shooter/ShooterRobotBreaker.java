// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakeAction;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterRobotBreaker extends Command {
  /** Creates a new ShooterRobotBreaker. */
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private LED led;
  private Timer timer = new Timer();
  public ShooterRobotBreaker(
    ShooterSubsystem shooterSubsystem,
    IntakeSubsystem intakeSubsystem,
    LED led
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.led = led;
    addRequirements(shooterSubsystem, intakeSubsystem, led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    led.setShooterBreaker(true);
    shooterSubsystem.setShooterRobotBreaker();;
    intakeSubsystem.setIntakeAction(IntakeAction.kShoote);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.StopMotor();
    intakeSubsystem.setIntakeAction(IntakeAction.kStop);
    led.setShooterBreaker(false);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(2.0);
  }
}
