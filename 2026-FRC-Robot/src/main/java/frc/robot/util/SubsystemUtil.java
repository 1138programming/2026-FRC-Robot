package frc.robot.util;

public class SubsystemUtil {

  private static double swerveSpeedFactor = 0.6;

  public SubsystemUtil() {}

  public void setSwerveMaxSpeed(double speedMod) {
    swerveSpeedFactor = speedMod;
  }

  public double getSwerveMaxSpeed() {
    return swerveSpeedFactor;
  }
}
