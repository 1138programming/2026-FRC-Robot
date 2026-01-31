// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  // defining
  TalonFX rotationMotor;
  TalonFX hoodMotor;
  TalonFX flywheelMotor;

  CANcoder turretRotationCANcoder;
  CANcoder hoodPitchCANcoder;

  PIDController rotationMotorPID;
  PIDController hoodMotorPID;
  PIDController flywheelMotorPID;
  final VelocityVoltage flywheelMotorRequest;

  /** Creates a new Turret. */
  public Turret() {
    
    // constructors
    rotationMotor = new TalonFX(KrotationMotorID);
    hoodMotor = new TalonFX(KhoodMotorID);
    flywheelMotor = new TalonFX(KflywheelMotorID);

    // Flywheel PID
    var flywheelConfig = new Slot0Configs();
    flywheelConfig.kP = KflywheelMotorP;
    flywheelConfig.kI = KflywheelMotorI;
    flywheelConfig.kD = KflywheelMotorD;
    flywheelMotorRequest = new VelocityVoltage(0).withSlot(0);
   
    flywheelMotor.getConfigurator().apply(flywheelConfig);

    turretRotationCANcoder = new CANcoder(KturretRotationCANcoderID);
    hoodPitchCANcoder = new CANcoder(KhoodPitchCANcoderID);

    rotationMotorPID = new PIDController(KrotationMotorkP, KrotationMotorkI, KrotationMotorkD);
    hoodMotorPID = new PIDController(KhoodMotorkP, KhoodMotorkI, KhoodMotorkP);

    rotationMotorPID.enableContinuousInput(-1.0, 1.0);
    hoodMotorPID.enableContinuousInput(-1.0, 1.0);
  }

  // ==================== MOTOR ROTATIONS ====================

  public void rotateRotationMotor(double power) { // rotates the main rotation motor of the turret
    double rotationDegree = getRotationDegree();
    // Make sure motor doesn't power when turret is outside of limits (0-270)
    if (rotationDegree >= KrotationMotorRightLim && rotationDegree <= KrotationMotorLeftLim) {
      // hard stop if it's outside the 270 degrees
      rotationMotor.set(0.0);
      return;
    }

    rotationMotor.set(power);
  }

  // Make sure motor doesn't power when hood is outside of limits (TBD)

  public void rotateHoodMotor(double power) {
    double hoodDegree = getHoodDegree();
    // outside limits
    if (hoodDegree >= KhoodMotorRightLim && hoodDegree <= KhoodMotorLeftLim) {
      hoodMotor.set(0.0);
      return;
    }

    hoodMotor.set(power);
  }

  // ==================== FLYWHEEL ====================

  public void setFlyWheelVelocity(double velocity) {
    flywheelMotor.setControl(flywheelMotorRequest.withVelocity(velocity).withFeedForward(0.5));
  }

  public double getFlywheelMotorVelocity() {
    return flywheelMotor.getVelocity().getValueAsDouble();
  }

  // ==================== MOTOR DEGREES ====================

  public double getRotationDegree() {
    return (turretRotationCANcoder.getAbsolutePosition().getValueAsDouble() - KrotationMotorOffset)
        * 360.0; // converts it to a degree
  }

  public double getHoodDegree() {
    return (hoodPitchCANcoder.getAbsolutePosition().getValueAsDouble() - KhoodMotorOffset) * 360;
  }

  // ==================== MOVE TO FUNCTIONS ====================

  public void rotationMoveToPosition(double degrees) {
    double rotationDegree = getRotationDegree();
    if (!(rotationDegree >= KrotationMotorRightLim && rotationDegree <= KrotationMotorLeftLim)) {
      rotateRotationMotor(rotationMotorPID.calculate(getRotationDegree(), degrees));
    }
  }

  public void hoodMoveToPosition(double degrees) {
    double hoodDegree = getHoodDegree();
    if (hoodDegree >= KhoodMotorRightLim && hoodDegree <= KhoodMotorLeftLim) {
      rotateHoodMotor(hoodMotorPID.calculate(getHoodDegree(), degrees));
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("rotation", getRotationDegree());
  }
}
