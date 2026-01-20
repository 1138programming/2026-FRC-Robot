// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.kHoodAngleMaxRadians;
import static frc.robot.Constants.TurretConstants.kHoodAngleMinRadians;

import java.util.Vector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drive.Drive;

import frc.robot.Constants;
import static frc.robot.Constants.TurretConstants.*;
import static frc.robot.Constants.FieldConstants.*;

public class ShooterLogic extends SubsystemBase {
  /** Creates a new ShooterLogic. */

  private Limelight limelight;
  private Drive drive;

  public ShooterLogic(Limelight limelight, Drive drive) {
    this.limelight = limelight;
    this.drive = drive;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * 
   * @param robotHeading
   * @return double[] {flywheelSpeed (meters per second), hoodAngle (radians), turretAngle (radians)}
   */
  public double[] calculateShotChanges(double robotHeading) {

    final double g = 32.174 * 12;
    double x = limelight.getHubCenterTagtoOffsetHubCenterDistancetoCamera() - kPassThroughPointRadius; //could be alternatively used using Pose
    double y = kScoreHeight; //could be alternatively used using Pose
    double a = kScoreAngle;

    //initial launch components
    double hoodAngle = Math.max(kHoodAngleMinRadians, Math.min(kHoodAngleMaxRadians, (Math.atan(2 * y / x - Math.tan(a))))); //this clamps the hood angle to constraints
    double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

    //robot velocity components
    double robotVelcity = drive.getFFCharacterizationVelocity();
    double robotAngle = drive.getPose().getRotation().getRadians();
    double robotVelocityXComponent = robotVelcity * Math.cos(robotAngle);
    double robotVelocityYComponent = robotVelcity * Math.sin(robotAngle);

    //velocity compensation variables
    double vz = flywheelSpeed * Math.sin(hoodAngle);
    double time = x / (flywheelSpeed * Math.cos(hoodAngle));
    double ivr = x / time + robotVelocityXComponent;
    double nvr = Math.sqrt(ivr * ivr + robotVelocityYComponent * robotVelocityYComponent);
    double ndr = nvr * time;

    //final launch components with compensation
    hoodAngle = Math.max(kHoodAngleMinRadians, Math.min(kHoodAngleMaxRadians, (Math.atan(vz / nvr))));
    flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));

    //updating turret
    double turretVelCompensation = Math.atan(robotVelocityYComponent / ivr);
    double turretAngle = (robotHeading - robotAngle + turretVelCompensation);

    if (turretAngle > Math.toRadians(180)) {
      turretAngle -= Math.toRadians(360);
    }

    return new double[] {flywheelSpeed, hoodAngle, turretAngle};
  }
}
