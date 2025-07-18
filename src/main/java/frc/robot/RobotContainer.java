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
import frc.robot.subsystems.DriverTrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants.ShooterAction;
import frc.robot.commands.Drive.DriveCommand;
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

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private static CommandXboxController m_driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);


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
  
    m_driverController.rightBumper().onTrue(new ShooterNormal(shooterSubsystem, ShooterAction.kShoote));
    m_driverController.leftBumper().onTrue(new ShooterNormal(shooterSubsystem, ShooterAction.kStop));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
  }

  private void setDefaultCommand() {
    driverTrain.setDefaultCommand(new DriveCommand(
      () -> -m_driverController.getLeftY() * 
          (m_driverController.getHID().getStartButton() ? 1 : 0.5), 
      () -> -m_driverController.getRightX(),
       driverTrain));
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
