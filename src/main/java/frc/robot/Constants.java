// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

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

    public static final double kcorlorSensorLGateValue = 200;

    public static final double kAngleMotorRatio = 0.0;

    public static final double kAngleUpLimit = 0.660;
    public static final double kAngleDownLimit = 0.800;

    //  Angle PID
    public static final double AnglekP = 3.0;
    public static final double AnglekI = 0.0;
    public static final double AnglekD = 0.0;
    public static final double AnglekIz = 0;
    public static final double AnglekFF = 0;
    public static final double AnglekMaxOutput = 0.5;
    public static final double AnglekMinOutput = -0.5;

    public enum AngleAction {
      kUP(0.2),
      kDown(-0.2),
      kStop(0);

      public final double rate;

      private AngleAction(double rate) {
        this.rate = rate;
      }
    }

    public enum IntakeState {
      kDefult(0.665),
      kGetBall(0.795);

      public final double position;

      private IntakeState(double position) {
        this.position = position;
      }
    }

    public enum IntakeAction {
      kGet(0.3),
      kShoote(0.9),
      kRev(-0.3),
      kStop(0);

      public final double rate;

      private IntakeAction(double rate) {
        this.rate = rate;
      }
    }
  
  }


  // Shooter
  public static final class ShooterConstants {
    public static final int SHOOTER_LEFT_MOTOR_PORT = 7;
    public static final int SHOOTER_RIGHT_MOTOR_PORT = 8;

    public static final double kLShoote = 0.7;
    public static final double kRShoote = 1.0;
    public static final double kStop = 0.0;

    // public enum ShooterAction {
    //   kLShoote(0.7),
    //   kRShoote(1),
    //   kStop(0);

    //   public final double rate;

    //   private ShooterAction(double rate){
    //     this.rate = rate;
    //   }
    // }
  }

  public static final class LEDConstants {
     // LED
    public static final int LED_LEFT_START = 0;
    public static final int LED_LEFT_END = 0;
    public static final int LED_RIGHT_START = 0;
    public static final int LED_RIGHT_END = 0;

    // LED Color
    public static final Color LED_RED = new Color(255, 0, 0);
    public static final Color LED_GREEN = new Color(0, 255, 0);
    public static final Color LED_BLUE = new Color(0, 0, 255);
    public static final Color LED_OFF = new Color(0, 0, 0);
    
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