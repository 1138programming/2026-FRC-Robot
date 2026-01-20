// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.kHoodAngleMaxRadians;
import static frc.robot.Constants.TurretConstants.kHoodAngleMinRadians;

import java.util.Vector;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants.TurretConstants.*;
import frc.robot.Constants.ShooterLogicConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.ShooterLogicConstants.*;

public class ShooterLogic extends SubsystemBase {
  /** Creates a new ShooterLogic. */

  private Limelight limelight;

  public ShooterLogic(Limelight limelight) {
    this.limelight = limelight;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void calculateShotChanges() {


    final double g = 32.174 * 12;
    double x = limelight.getHubCenterTagtoOffsetHubCenterDistancetoCamera() - FieldConstants.kPassThroughPointRadius; //could be alternatively used using Pose
    double y = FieldConstants.kScoreHeight;
    double a = FieldConstants.kScoreAngle;

    //initial launch components
    double hoodAngle = Math.max(kHoodAngleMinRadians, Math.min(kHoodAngleMaxRadians, (Math.atan(2 * y / x - Math.tan(a))))); //this clamps the hood angle to constraints
    double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));
    
    SwerveDriveOdometry EXAMPLE_ODOM = new ChassisSpeeds(); //PLACEHOLDER FOR CODE TO WORK USE ACTUAL DATA FROM DRIVE
    Vector robotVelocity =

    double coordinateTheta = robotVelocity.

  }
}
