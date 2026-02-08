// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.kHoodAngleMaxRadians;
import static frc.robot.Constants.TurretConstants.kHoodAngleMinRadians;

import java.util.Vector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drive.Drive;

import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.HubConstants;
import frc.robot.Constants.TurretConstants.TurretOffsetConstants;

import static frc.robot.Constants.TurretConstants.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.FieldConstants.HubConstants.*;
import static frc.robot.Constants.TurretConstants;

public class ShooterLogic extends SubsystemBase {
  /** Creates a new ShooterLogic. */

  private Limelight limelight;
  private Drive drive;
  private Turret turret;

  private Pose3d turretPose3d;
  private Pose2d turretPose2d;

  public ShooterLogic(Limelight limelight, Drive drive, Turret turret) {
    this.limelight = limelight;
    this.drive = drive;
    this.turret = turret;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //update turret pose
    turretPose3d = turretPositionPose3d();
    turretPose2d = turretPose3d.toPose2d();
    // addTurretRotationtoPose();

    //SmartDashboard.putNumber("TX Helper", absoluteAngletoAprilTagLimelightDegrees(0));
    SmartDashboard.putString("Turret Pose 3d", turretPose3d.toString());
    SmartDashboard.putString("Turret Pose 2d", turretPose2d.toString());
    SmartDashboard.putString("diff translation", Constants.FieldConstants.HubConstants.kHubFieldPose2d.getTranslation().minus(turretPose2d.getTranslation()).toString());

    SmartDashboard.putNumber("Distance to Hub Center", distancetoPose2d(Constants.FieldConstants.HubConstants.kHubFieldPose2d));
    SmartDashboard.putNumber("Angle to Hub Center", turretAngletoPose2d(Constants.FieldConstants.HubConstants.kHubFieldPose2d));
    SmartDashboard.putNumber("turret angle output", relativeTurretAngletoPos(Constants.FieldConstants.HubConstants.kHubFieldPose2d));

  }

  /**
   * Calculates the flywheel speed, hood angle, and turret angle based on robot position in accordance to the hub center with velocity compensation.
   * 
   * @return double[] {flywheelSpeed (meters per second), hoodAngle (radians), turretAngle (radians)}
   */

  public double[] calculateShotChanges() {

    final double g = 9.81;
    double x =  distancetoPose2d(HubConstants.kHubFieldPose2d) - kPassThroughPointRadius; //could be alternatively used using Pose
    double y = kScoreHeight; //could be alternatively used using Pose
    double a = kScoreAngle;
    double robotAngle = drive.getRotation().getRadians(); //robot angle in reference to field
    double robottoGoalAngle = turretAngletoPose2d(HubConstants.kHubFieldPose2d); //angle from robot to goal in reference to field

    //initial launch components
    double hoodAngle = Math.max(kHoodAngleMinRadians, Math.min(kHoodAngleMaxRadians, (Math.atan(2 * y / x - Math.tan(a))))); //this clamps the hood angle to constraints
    double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

    //robot velocity components -> TODO, check video to see if this matches up
    //double robotVelocity = drive.getchas(); //TODO: probably get the velocity from the IMU, also check units  
    double robotVelocityXComponent = drive.getHorizontalVelocityMetersPerSecond();
    double robotVelocityYComponent = drive.getVerticalVelocityMetersPerSecond();

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
    double turretAngle = (robotAngle - robottoGoalAngle) + turretVelCompensation;//TODO check signs especially for turret compensation

  
    if (turretAngle > Math.toRadians(180)) {
      turretAngle -= Math.toRadians(360);
    }

    return new double[] {flywheelSpeed, hoodAngle, turretAngle};
  }

  /**
   * 
   * Used for early autoaiming to start orienting the turret until shotchanges can be calculated
   * can also be used for updating turret angle if limelight is the only source of aiming.
   * Will require future implementation
   * @return suggested absolute angle to aim turret (radians)
   */
  public double absoluteAngletoAprilTagLimelightRadians() {
    double angleDif = limelight.getTx();
    double absoluteAngle = turret.getTurretRotationDegree() + angleDif;
    return absoluteAngle;
  }

  public double absoluteAngletoAprilTagLimelightDegrees() {
    return Math.toDegrees(absoluteAngletoAprilTagLimelightRadians());
  }

  public double absoluteAngletoAprilTagLimelightDegrees(double limelightOffsetAngleDegrees) {
    double angleDif = limelight.getTxHelper();
    double absoluteAngle = limelightOffsetAngleDegrees + angleDif;
    return absoluteAngle;
  }
  public double relativeTurretAngletoPos( Pose2d pose) {
    double angle = turretAngletoPose2d(pose);
    if(angle > 180) {
      angle -= 360;
    } 
    if(angle < -180) {
      angle += 360;
    } 
    return angle;
    
  }



  /**
   * @return angle difference to aim turret (radians); the Tx value from limelight
   */
  public double getAngletoAprilTageLimelight() {
    double angleDif = limelight.getTx();
    return Math.toRadians(angleDif);
  }

  //in shooter logic as it requires continual adjustment by drive for the robot's position
  //Review if this is alright here
  private Pose3d turretPositionPose3d() {

    Rotation3d currentRotation = new Rotation3d(drive.getRotation());
    Pose3d currentPose = new Pose3d(drive.getPose().getX(), drive.getPose().getY(), 0.0, currentRotation);
    
    Translation3d translationOffset = new Translation3d(TurretOffsetConstants.kForwardOffsetMeters_X, TurretOffsetConstants.kSideOffsetMeters_Y, TurretOffsetConstants.kVerticalOffsetMeters_Z); //include turret offsets once known. Placeholder is top right corner of robot
    Rotation3d rotationOffset = new Rotation3d(TurretOffsetConstants.kTurretYawOffsetRadians, TurretOffsetConstants.kTurretPitchOffsetRadians,TurretOffsetConstants.kTurretYawOffsetRadians - Math.toRadians(turret.getTurretRotationDegree()));
    Transform3d offsetTransformation = new Transform3d(translationOffset, rotationOffset);

    return currentPose.plus(offsetTransformation);
  }

  // private void addTurretRotationtoPose() {
  //   Rotation3d currentRotation = turretPose3d.getRotation();
  //   Rotation3d newRotation = new Rotation3d(currentRotation.getMeasureX().magnitude(), currentRotation.getMeasureY().magnitude(), currentRotation.getMeasureZ().magnitude() - Math.toRadians(turret.getTurretRotationDegree())); //negative turret angle because pose is CCW + and turret angle is CW +
  //   turretPose3d.rotateBy(newRotation);
    
  // } 

  private double distancetoPose2d(Pose2d pose2d) {
    return turretPose2d.getTranslation().getDistance(pose2d.getTranslation());
  }

  private double distancetoPose3d(Pose3d pose3d) {
    return turretPose3d.getTranslation().getDistance(pose3d.getTranslation());
  }

  private double turretAngletoPose2d(Pose2d pose2d) {
    Translation2d difftranslation = pose2d.getTranslation().minus(turretPose2d.getTranslation());
    // if(difftranslation.getAngle().getDegrees() > 180) {
    //   return drive.getRotation().getDegrees() - ( difftranslation.getAngle().getDegrees() - 360); //90 because turret 0 is 90 degrees according to field
    // } 
  
    return drive.getRotation().getDegrees() - difftranslation.getAngle().getDegrees(); //90 because turret 0 is 90 degrees according to field

    //return Math.toDegrees(Math.atan(difftranslation.getY() / difftranslation.getX()));   // pose2d.getTranslation().minus(turretPose2d.getTranslation()).getAngle().getRadians();
  }

}



