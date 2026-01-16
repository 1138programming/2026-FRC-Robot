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
  TalonFX rotationMotor;
  TalonFX yawMotor;
  CANcoder turretRotationCANcoder;
  
  PIDController rotationMotorPID;

  /** Creates a new Turret. */
  public Turret() {
    rotationMotor = new TalonFX(0);
    turretRotationCANcoder = new CANcoder(1);
    rotationMotorPID = new PIDController(rotationMotorkP, rotationMotorkI, rotationMotorkD);
    rotationMotorPID.enableContinuousInput(-1.0, 1.0);
  }

  public void rotateMotor(double power) { //rotates the main rotation motor of the turret

    //Tracking values outside the 270 degree turn of the turret.
    if (getRotationDegree() >= rotationMotorRightLim && getRotationDegree() <= rotationMotorLeftLim) {
      rotationMotor.set(0.0);
    }

    rotationMotor.set(power);
  }

  public double getRotationDegree() { 
    return (turretRotationCANcoder.getAbsolutePosition().getValueAsDouble() - rotationMotorOffSet) * 360.0; //converts it to a degree
  }

  public void moveToPosition(double degrees) {
    if (!(getRotationDegree() >= rotationMotorRightLim && getRotationDegree() <= rotationMotorLeftLim)) {
      rotateMotor(rotationMotorPID.calculate(getRotationDegree(), degrees));
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
