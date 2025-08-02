// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.DriverTrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.IntakeConstants.AngleAction;
import frc.robot.Constants.IntakeConstants.IntakeAction;
import frc.robot.Constants.IntakeConstants.IntakeState;
// import frc.robot.Constants.ShooterConstants.ShooterAction;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.Intake.IntakeAuto;
import frc.robot.commands.Intake.IntakeNormal;
import frc.robot.commands.Intake.IntakeStateAuto;
import frc.robot.commands.Intake.IntakeStateNormal;
import frc.robot.commands.Shooter.ShooterAuto;
import frc.robot.commands.Shooter.ShooterNormal;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriverTrain driverTrain = new DriverTrain();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final AngleSubsystem angleSubsystem = new AngleSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private static CommandXboxController m_driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);
  private static CommandXboxController m_operatorController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);


  // Create auto chooser
  // private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    // configureNamedCommands();
    // autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    // SmartDashboard.putData("Auto Mode", autoChooser);
    configureBindings();
    setDefaultCommand();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Intake state normal
    m_driverController.pov(0).whileTrue(new IntakeStateNormal(angleSubsystem, AngleAction.kUP));
    m_driverController.pov(180).whileTrue(new IntakeStateNormal(angleSubsystem, AngleAction.kDown));
    
    m_driverController.leftBumper().onTrue(new IntakeStateAuto(angleSubsystem, IntakeState.kDefult));
    m_driverController.rightBumper().onTrue(new IntakeStateAuto(angleSubsystem, IntakeState.kGetBall));

    // Intake normal
    m_driverController.x().whileTrue(new IntakeNormal(intakeSubsystem, IntakeAction.kGet));
    m_driverController.y().whileTrue(new IntakeNormal(intakeSubsystem, IntakeAction.kRev));

    // Intake Auto
    m_driverController.a().onTrue(new IntakeAuto(intakeSubsystem, IntakeAction.kGet, angleSubsystem));

    // shooter normal
    m_operatorController.b().toggleOnTrue(new ShooterNormal(shooterSubsystem));
    // m_operatorController.leftBumper().onTrue(new ShooterNormal(shooterSubsystem));

    // Shooter Auto
    m_operatorController.a().onTrue(new ShooterAuto(shooterSubsystem, intakeSubsystem));
  }

  private void setDefaultCommand() {
    driverTrain.setDefaultCommand(new DriveCommand(
      driverTrain,
      () -> m_driverController.getLeftY() * 
          0.5, 
      () -> m_driverController.getRightX() * 0.5));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
