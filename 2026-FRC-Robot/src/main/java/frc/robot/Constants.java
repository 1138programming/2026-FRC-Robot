// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
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
    
  public static class LaserConstants {
    public static final int kLaserDIOPort = 3;
  }
}
