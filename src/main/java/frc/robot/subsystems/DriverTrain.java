package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class DriverTrain extends SubsystemBase {
  private final SparkMax FRONT_LEFT_MOTOR = new SparkMax(DriveConstants.FRONT_LEFT_MOTOR_PORT, MotorType.kBrushless);
  private final SparkMax FRONT_RIGHT_MOTOR = new SparkMax(DriveConstants.FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushless);
  private final SparkMax BACK_LEFT_MOTOR = new SparkMax(DriveConstants.BACK_LEFT_MOTOR_PORT, MotorType.kBrushless);
  private final SparkMax BACK_RIGHT_MOTOR = new SparkMax(DriveConstants.BACK_RIGHT_MOTOR_PORT, MotorType.kBrushless);

  private final SparkMaxConfig FRONT_LEFT_CONFIG = new SparkMaxConfig();
  private final SparkMaxConfig FRONT_RIGHT_CONFIG = new SparkMaxConfig();
  private final SparkMaxConfig BACK_LEFT_CONFIG = new SparkMaxConfig();
  private final SparkMaxConfig BACK_RIGHT_CONFIG = new SparkMaxConfig();

  private final DifferentialDrive drive = new DifferentialDrive(FRONT_LEFT_MOTOR, FRONT_RIGHT_MOTOR);

  private final static AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  private final Field2d field = new Field2d();
  private RobotConfig robotConfig;
  private boolean isRED = false;
  public static double heading;

  private final RelativeEncoder leftEncoder = FRONT_LEFT_MOTOR.getEncoder();
  private final RelativeEncoder rightEncoder = FRONT_RIGHT_MOTOR.getEncoder();

  public DriverTrain() {
    // wait a bit then zero gyro to let navX initialize safely
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }).start();

    // Motor configs (包含 encoder conversion 在 config 裡)
    FRONT_LEFT_CONFIG
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60);

    FRONT_RIGHT_CONFIG
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60);

    BACK_LEFT_CONFIG.follow(FRONT_LEFT_MOTOR);
    BACK_RIGHT_CONFIG.follow(FRONT_RIGHT_MOTOR);

    FRONT_LEFT_MOTOR.configure(FRONT_LEFT_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    FRONT_RIGHT_MOTOR.configure(FRONT_RIGHT_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    BACK_LEFT_MOTOR.configure(BACK_LEFT_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    BACK_RIGHT_MOTOR.configure(BACK_RIGHT_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    drive.setDeadband(0.05);


   
    
    SmartDashboard.putData("Field", field);

   
    // resetOdometry(new Pose2d());
  }

  @Override
  public void periodic() {
  }

  

  /* ---------- Drive methods ---------- */
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(DriveConstants.kFedPercent * xSpeed, DriveConstants.kRotPercent * zRotation);
  }

  

  /* ---------- Gyro / Odometry helpers ---------- */
  // Reset gyro heading
  public void zeroHeading() {
    gyro.reset();
    heading = getHeading();
  }

  // Return gyro heading (degrees)
  public static double getHeading() {
    return gyro.getAngle();
  }

  public Rotation2d getOdometryAngle() {
    return Rotation2d.fromDegrees(getRobotDegrees() - 180.0);
  }

  public double getRobotDegrees() {
    double rawValue = -gyro.getAngle() % 360.0;
    if (rawValue < 0.0) {
      return (rawValue + 360.0);
    } else {
      return (rawValue);
    }
  }

  

   
}
