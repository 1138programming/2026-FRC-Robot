// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.generated.TunerConstants.kSpeedAt12Volts;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class SwerveConstants {

    // public static final double KMaxSpeed = kSpeedAt12Volts.in(MetersPerSecond);
    // // kSpeedAt12Volts desired
    public static final double KMaxSpeed =
        kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
    // top speed

    public static final double KMaxAngularRate =
        RotationsPerSecond.of(1.8).in(RadiansPerSecond); // 3/4 of
    // a
    // rotation
    // per
    // second
    // max angular velocity

    public static final double KBaseTurboMode = 1;
    public static final double KBaseNormalMode = 0.5;
    public static final double KBaseSlowMode = 0.25;
  }

  public static class OperatorConstants {
        // Controller Ports (check in Driver Station, IDs may be different for each
    // compStreamDeckuter)
    public static final int KLogitechPort = 0;
    public static final int KXboxPort = 1;
    public static final int KCompStreamDeckPort = 2;
    public static final int KTestingStreamDeckPort = 3;
    public static final int KAutonTestingStreamDeckPort = 4;

    // Deadzone
    public static final double KDeadZone = 0.05;

    // Joystick Axis IDs
    public static final int KLeftXAxis = 0;
    public static final int KLeftYAxis = 1;
    public static final int KRightXAxis = 2;
    public static final int KRightYAxis = 3;

    // Joystick Axis IDs
    public static final int KXboxLeftYAxis = 1;
    public static final int KXboxRightYAxis = 5;
    public static final int KXboxLeftXAxis = 0;
    public static final int KXboxRightXAxis = 4;

    // Logitech Button Constants
    public static final int KLogitechButtonX = 1;
    public static final int KLogitechButtonA = 2;
    public static final int KLogitechButtonB = 3;
    public static final int KLogitechButtonY = 4;
    public static final int KLogitechLeftBumper = 5;
    public static final int KLogitechRightBumper = 6;
    public static final int KLogitechLeftTrigger = 7;
    public static final int KLogitechRightTrigger = 8;
    public static final int KLogitechBtnBack = 9;
    public static final int KLogitechRightStart = 10;

    // Xbox Button Constants
    public static final int KXboxButtonA = 1;
    public static final int KXboxButtonB = 2;
    public static final int KXboxButtonX = 3;
    public static final int KXboxButtonY = 4;
    public static final int KXboxLeftBumper = 5;
    public static final int KXboxRightBumper = 6;
    public static final int KXboxSelectButton = 7;
    public static final int KXboxStartButton = 8;
    public static final int KXboxLeftTrigger = 2;
    public static final int KXboxRightTrigger = 3;
    public static final int kDriverControllerPort = 0;
  }

  public static class TurretConstants {
    public static final double KrotationMotorkP = 20.0;
    public static final double KrotationMotorkI = 0.0;
    public static final double KrotationMotorkD = 0.0;

    public static final double KhoodMotorkP = 20.0;
    public static final double KhoodMotorkI = 0.0;
    public static final double KhoodMotorkD = 0.0;

    public static final double KflywheelMotorP = 2.4;
    public static final double KflywheelMotorI = 0.0;
    public static final double KflywheelMotorD = 0.1;

    public static final double KrotationMotorOffset = 0.0;
    public static final double KhoodMotorOffset = 0.0;

    public static final double KrotationMotorRightLim = 130.0;
    public static final double KrotationMotorLeftLim = 230.0;
    public static final double KhoodMotorRightLim = 90.0;
    public static final double KhoodMotorLeftLim = 0.0;

    public static final int KrotationMotorID = 0;
    public static final int KhoodMotorID = 2;
    public static final int KflywheelMotorID = 4;

    public static final int KturretRotationCANcoderID = 1;
    public static final int KhoodPitchCANcoderID = 3;
  

  }
  public static class LimelightConstants {
    public static class TagConstants {
      public static class IDConstants {
        public static final int kHubCenterTagBlue = 26;
        public static final int kHubCenterTagRed = 10;
      }

      public static class OffsetConstants {
        public static final double kX_HubCenterTagtoHubCenterMeters = -0.5969;
        public static final double kY_HubCenterTagtoHubScoreHeightMeters = 0.70485;
      }
    }
  }

  public static class LaserConstants {
    public static final int klaserDIOPort = 3;
  }
}
