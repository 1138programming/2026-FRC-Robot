// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.TurretConstants.*;

public class Turret extends SubsystemBase {
  //defining
  TalonFX rotationMotor;
  TalonFX hoodMotor;
  CANcoder turretRotationCANcoder;
  CANcoder hoodPitchCANcoder;
  PIDController rotationMotorPID;
  PIDController hoodMotorPID;

  /** Creates a new Turret. */
  public Turret() {
    //creation of everything
    rotationMotor = new TalonFX(rotationMotorID);
    hoodMotor = new TalonFX(hoodMotorID);

    turretRotationCANcoder = new CANcoder(turretRotationCANcoderID);
    hoodPitchCANcoder = new CANcoder(hoodPitchCANcoderID);

    rotationMotorPID = new PIDController(rotationMotorkP, rotationMotorkI, rotationMotorkD);
    hoodMotorPID = new PIDController(hoodMotorkP, hoodMotorkI, hoodMotorkP);
    
    rotationMotorPID.enableContinuousInput(-1.0, 1.0);
    hoodMotorPID.enableContinuousInput(-1.0, 1.0);
  }


  //==================== MOTOR ROTATIONS ====================
  public void rotateRotationMotor(double power) { //rotates the main rotation motor of the turret
    double rotationDegree = getRotationDegree();
    //Make sure motor doesn't power when turret is outside of limits (0-270)
    if (rotationDegree >= rotationMotorRightLim && rotationDegree <= rotationMotorLeftLim) {
      //hard stop if it's outside the 270 degrees
      rotationMotor.set(0.0);
      return;
    }
    
    rotationMotor.set(power);
  }

  //Make sure motor doesn't power when hood is outside of limits (TBD)

  public void rotateHoodMotor(double power) {
    double hoodDegree = getHoodDegree();
    if (hoodDegree >= hoodMotorRightLim && hoodDegree <= hoodMotorLeftLim) {
      hoodMotor.set(0.0);
    }
    hoodMotor.set(0.0);
  }


  //==================== MOTOR DEGREES ====================
  public double getRotationDegree() { 
    return (turretRotationCANcoder.getAbsolutePosition().getValueAsDouble() - rotationMotorOffset) * 360.0; //converts it to a degree
  }

  public double getHoodDegree() {
    return (hoodPitchCANcoder.getAbsolutePosition().getValueAsDouble() - hoodMotorOffset) * 360;
  }

  //==================== MOVE TO FUNCTIONS ====================
  public void rotationMoveToPosition(double degrees) {
    double rotationDegree = getRotationDegree();
    if (!(rotationDegree >= rotationMotorRightLim && rotationDegree <= rotationMotorLeftLim)) {
      rotateRotationMotor(rotationMotorPID.calculate(getRotationDegree(), degrees));
    }
  }

  public void hoodMoveToPosition(double degrees) {
    double hoodDegree = getHoodDegree();
    if (hoodDegree >= hoodMotorRightLim && hoodDegree <= hoodMotorLeftLim) {
      rotateHoodMotor(hoodMotorPID.calculate(getHoodDegree(), degrees));
    }
  }
  
  
  //==================== PERIODIC ====================
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
