// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;
import static frc.robot.Constants.HangConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Hang extends SubsystemBase {
  private Servo linearActuator1, linearActuator2;
  private TalonFX rotationMotor1, rotationMotor2;
  private SparkMax leadScrewRotation;
  /** Creates a new Hang. */
  public Hang() {
    linearActuator1 = new Servo(klinearActuator1ID);
    linearActuator1 = new Servo(klinearActuator2ID);
    rotationMotor1 = new TalonFX(kRotationMotor1ID);
    rotationMotor2 = new TalonFX(kRotationMotor2ID);
    leadScrewRotation = new SparkMax(kRotationMotor1ID, MotorType.kBrushless);
  }

  public void setActuatorPositions(double position) {
    linearActuator1.setPosition(position);
    linearActuator2.setPosition(position);
  }

  public void spinRotationMotor(double power) {
    rotationMotor1.set(power);
    rotationMotor2.set(power);
  }

  public void spinLeadScrew(double power) {
    leadScrewRotation.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
