// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static frc.robot.Constants.TurretConstants.*;

public class Turret extends SubsystemBase {
  //defining
  SparkMax rotationMotor;
  TalonFX hoodMotor;
  TalonFX flywheelMotor;

  CANcoder turretRotationCANcoder;
  CANcoder hoodPitchCANcoder;

  PIDController rotationMotorPID;
  PIDController hoodMotorPID;
  PIDController flywheelMotorPID;
  final VelocityVoltage flywheelMotorRequest;
  DigitalInput leftLimSwitch;
  DigitalInput rightLimSwitch;

  double previousDegree = getRotationDegree();
  static double CANRotatedDegrees = 0.0;
  double currentDegree;
  double currentRotationPower;
  double turretDegree;
  
  /** Creates a new Turret. */
  public Turret() {
    //Flywheel PID
    var flywheelConfig = new Slot0Configs();
    flywheelConfig.kP = KflywheelMotorP;
    flywheelConfig.kI = KflywheelMotorI;
    flywheelConfig.kD = KflywheelMotorD;
    flywheelMotorRequest = new VelocityVoltage(0).withSlot(0);

    
    //constructors
    flywheelMotor.getConfigurator().apply(flywheelConfig);

    rotationMotor = new SparkMax(KrotationMotorID, MotorType.kBrushless);
    hoodMotor = new TalonFX(KhoodMotorID);
    flywheelMotor = new TalonFX(KflywheelMotorID);

    turretRotationCANcoder = new CANcoder(KturretRotationCANcoderID);
    hoodPitchCANcoder = new CANcoder(KhoodPitchCANcoderID);

    rotationMotorPID = new PIDController(KrotationMotorkP, KrotationMotorkI, KrotationMotorkD);
    hoodMotorPID = new PIDController(KhoodMotorkP, KhoodMotorkI, KhoodMotorkP);
    
    rotationMotorPID.enableContinuousInput(-1.0, 1.0);
    hoodMotorPID.enableContinuousInput(-1.0, 1.0);

    leftLimSwitch = new DigitalInput(KleftLimSwitchID);
    rightLimSwitch = new DigitalInput(KrightLimSwitchID);
  }

  public boolean getLeftLimitSwitchVal(){
    return leftLimSwitch.get();
  }

  public boolean getRightLimitSwitchVal(){
    return rightLimSwitch.get();
  }


  //==================== MOTOR ROTATIONS ====================
  
  public void updateTurretDeg() {
    currentDegree = getRotationDegree();
    currentRotationPower = getRotationMotorPower();

    if (getLeftLimitSwitchVal()) {
      CANRotatedDegrees = KrotationMotorLeftLim * kturretRotationstoMotorRotationCount;
      return;
    }

    if (getRightLimitSwitchVal()){
      CANRotatedDegrees = KrotationMotorRightLim * kturretRotationstoMotorRotationCount;
      return;
    }

    if (currentRotationPower == 0) return;

    if (currentRotationPower > 0) { //assume going clockwise if true
      if (currentDegree < previousDegree) { //did wrap around if true
        CANRotatedDegrees += (360 - previousDegree) + currentDegree; //add wrapped around difference
      } else {
        CANRotatedDegrees += currentDegree - previousDegree; //didn't wrap around so just add difference
      }
    } else {
      if (currentDegree > previousDegree){
        CANRotatedDegrees += previousDegree + (360 - currentDegree);
      } else {
        CANRotatedDegrees += currentDegree - previousDegree;
      }
    }
    previousDegree = currentDegree;

    turretDegree = Math.round(currentDegree/55);
  }

  //UPDATE

  public void rotateRotationMotor(double power) { //rotates the main rotation motor of the turret
    updateTurretDeg();
    //Make sure motor doesn't power when turret is outside of limits (0-270)
    if (turretDegree >= KrotationMotorRightLim && turretDegree <= KrotationMotorLeftLim) {
      rotationMotor.set(0.0);
      return;
    }
    
    rotationMotor.set(power);
  }
  

  //Make sure motor doesn't power when hood is outside of limits (TBD)

  public void rotateHoodMotor(double power) {
    double hoodDegree = getHoodDegree();
    //outside limits
    if (hoodDegree >= KhoodMotorRightLim && hoodDegree <= KhoodMotorLeftLim) {
      hoodMotor.set(0.0);
      return;
    }

    hoodMotor.set(power);
  }

  public double getTurretRotationInDegrees(){
    return CANRotatedDegrees / kturretRotationstoMotorRotationCount;
  }

  //==================== FLYWHEEL ====================

  public void setFlyWheelVelocity(double velocity) {
    flywheelMotor.setControl(flywheelMotorRequest.withVelocity(velocity).withFeedForward(0.5));
  }

  public double getFlywheelMotorVelocity() {
    return flywheelMotor.getVelocity().getValueAsDouble();
  }

  //==================== MOTOR DEGREES ====================

  public double getRotationDegree() { 
    return ((turretRotationCANcoder.getAbsolutePosition().getValueAsDouble() - KrotationMotorOffset) * 180.0) + 180; //converts it to a degree
  }

  public double getHoodDegree() {
    return (hoodPitchCANcoder.getAbsolutePosition().getValueAsDouble() - KhoodMotorOffset) * 360;
  }

  public double getRotationMotorPower(){
    return rotationMotor.get();
  }

  //==================== MOVE TO FUNCTIONS ====================

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
    SmartDashboard.putNumber("rotation of motor", getRotationDegree());
    SmartDashboard.putNumber("rotation of turret",getTurretRotationInDegrees());
    SmartDashboard.putBoolean("Left lim switch", getLeftLimitSwitchVal());
    SmartDashboard.putBoolean("Right lim switch", getRightLimitSwitchVal());
  }
}
