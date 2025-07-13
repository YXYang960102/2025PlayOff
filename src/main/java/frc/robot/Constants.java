// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // OIController
  public static class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final boolean kDriverFieldOriented = true;
    public static final double kDeadband = 0.05;

    public static double deadbandHandler(double value, double deadband) {
      if (Math.abs(value) < deadband) {
        return 0;
      } else if (value > 0) {
        return (value - OIConstants.kDeadband) / (1 - OIConstants.kDeadband);
      } else {
        return (value + OIConstants.kDeadband) / (1 - OIConstants.kDeadband);
      }
    }
  }

  // Drive Train
  public static class DriveConstants {
    public static final int FRONT_LEFT_MOTOR_PORT = 1;
    public static final int FRONT_RIGHT_MOTOR_PORT = 2;
    public static final int BACK_LEFT_MOTOR_PORT = 3;
    public static final int BACK_RIGHT_MOTOR_PORT = 4;
  }

  // Intake
  public static class IntakeConstants {
    public static final int ANGLE_MOTOR_PORT = 5;
    public static final int Intake_MOTOR_PORT = 6;

    public static final double kAngleUpLimit = 0;
    public static final double kAngleDownLimit = 0;

    // Algae Angle PID
    public static final double AlgaekP = 10.5;
    public static final double AlgaekI = 0.0;
    public static final double AlgaekD = 0.0;
    public static final double AlgaekIz = 0;
    public static final double AlgaekFF = 0;
    public static final double AlgaekMaxOutput = 1;
    public static final double AlgaekMinOutput = -1;

    public enum AlgaeGrabberState {
      kDefult(0),
      kIndex(0),
      kGet(0);
      

      public final double position;
  
      private AlgaeGrabberState(double position) {
        this.position = position;
      }

    public enum IntakeAction {
      kUP(0.5),
      kDown(-0.5),
      kStop(0);

      public final double rate;

      private IntakeAction(double rate) {
        this.rate = rate;
      }
    
  }

  

  // Limelight
  public static final class LimelightConstants {

    // Limelight Name Mapping
    public enum Limelight {
      // kCoral("limelight-b", 1), // IP: 10.81.69.13
      kReef("limelight-c", 1); // IP: 10.81.69.15

      public final String hostname;
      public final double approachingXSpeed;

      private Limelight(String hostname, double approachingXSpeed) {
        this.hostname = hostname;
        this.approachingXSpeed = approachingXSpeed;
      }
    }
  }


}
