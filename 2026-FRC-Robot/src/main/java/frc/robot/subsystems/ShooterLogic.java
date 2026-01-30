// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.kHoodAngleMaxRadians;
import static frc.robot.Constants.TurretConstants.kHoodAngleMinRadians;

import java.util.Vector;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drive.Drive;

import frc.robot.Constants;
import static frc.robot.Constants.TurretConstants.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.FieldConstants.HubConstants.*;
import static frc.robot.Constants.TurretConstants;

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
   * Calculates the flywheel speed, hood angle, and turret angle based on robot position in accordance to the hub center.
   * 
   * @param robotHeading 
   * @return double[] {flywheelSpeed (meters per second), hoodAngle (radians), turretAngle (radians)}
   */
  public double[] calculateShotChanges(double robotHeading) {

    final double g = 9.81;
    double x =  distanetoCenterHub() - kPassThroughPointRadius; //could be alternatively used using Pose
    double y = kScoreHeight; //could be alternatively used using Pose
    double a = kScoreAngle;

    //initial launch components
    double hoodAngle = Math.max(kHoodAngleMinRadians, Math.min(kHoodAngleMaxRadians, (Math.atan(2 * y / x - Math.tan(a))))); //this clamps the hood angle to constraints
    double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

    //robot velocity components -> TODO, check video to see if this matches up
    double robotVelocity = drive.getFFCharacterizationVelocity(); //TODO: probably get the velocity from the IMU, also check units  
    double robotAngle = drive.getPose().getRotation().getRadians();
    double robotVelocityXComponent = robotVelocity * Math.cos(robotAngle);
    double robotVelocityYComponent = robotVelocity * Math.sin(robotAngle);

    //velocity compensation variables
    double vz = flywheelSpeed * Math.sin(hoodAngle); //velocity of the projectile in z direction (vertical)
    double time = x / (flywheelSpeed * Math.cos(hoodAngle)); //projectile air time
    double ivr = x / time + robotVelocityXComponent; //initial radial velocity of of the projectile
    double nvr = Math.sqrt(ivr * ivr + robotVelocityYComponent * robotVelocityYComponent); //compensating launch velocity using perpendicular moevement 
    double ndr = nvr * time; //convert to distance

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

    //to be implemented into shooter logic most likely
  private Pose3d turretPositionPose3d() {

    Rotation3d currentRotation = new Rotation3d(drive.getRotation());
    Pose3d currentPose = new Pose3d(drive.getPose().getX(), drive.getPose().getY(), 0.0, currentRotation);
    
    Translation3d translationOffset = new Translation3d(TurretOffsetConstants.kForwardOffsetMeters_X, TurretOffsetConstants.kSideOffsetMeters_Y, TurretOffsetConstants.kVerticalOffsetMeters_Z); //include turret offsets once known. Placeholder is top right corner of robot
    Rotation3d rotationOffset = new Rotation3d(TurretOffsetConstants.kTurretYawOffsetRadians, TurretOffsetConstants.kTurretPitchOffsetRadians,TurretOffsetConstants.kTurretYawOffsetRadians);
    Transform3d offsetTransformation = new Transform3d(translationOffset, rotationOffset);

    return currentPose.plus(offsetTransformation);
  }

  private double distanetoCenterHub() {
    Pose3d currentPose = turretPositionPose3d();
    
    Pose3d hubCenterTop = new Pose3d(HubConstants.kPoseX, HubConstants.kPoseY, HubConstants.kPoseZ, new Rotation3d());
    double distance = currentPose.getTranslation().getDistance(hubCenterTop.getTranslation());
    return distance;
  }
}



