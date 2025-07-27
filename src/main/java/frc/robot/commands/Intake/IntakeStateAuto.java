// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.subsystems.AngleSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeStateAuto extends InstantCommand {
  private AngleSubsystem angleSubsystem;
  private IntakeState intakeState;

  public IntakeStateAuto(
    AngleSubsystem angleSubsystem,
    IntakeState intakeState
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angleSubsystem = angleSubsystem;
    this.intakeState = intakeState;
    addRequirements(angleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   if(intakeState != null){
    angleSubsystem.setState(intakeState);
   }
  }
}
