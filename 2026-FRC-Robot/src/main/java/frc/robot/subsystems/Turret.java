// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Turret extends SubsystemBase {

  //defining
  private SparkMax rotationMotor;

  private TalonFX hoodMotor;
  private TalonFX flywheelMotor;

  private CANcoder turretRotationCANcoder;
  private CANcoder hoodPitchCANcoder;
  private CANcoderConfiguration rotationCANcoderConfig; 
  private PIDController rotationMotorPID;
  private PIDController hoodMotorPID;
  private PIDController flywheelMotorPID;
  private final VelocityVoltage flywheelMotorRequest;
  private DigitalInput leftLimSwitch;
  private DigitalInput rightLimSwitch;

  private double turretOffset;
  
  /** Creates a new Turret. */
  public Turret() {
    
    // constructors
    hoodMotor = new TalonFX(KhoodMotorID);
    flywheelMotor = new TalonFX(KflywheelMotorID);

    // Flywheel PID
    var flywheelConfig = new Slot0Configs();
    flywheelConfig.kP = KflywheelMotorP;
    flywheelConfig.kI = KflywheelMotorI;
    flywheelConfig.kD = KflywheelMotorD;
    flywheelMotorRequest = new VelocityVoltage(0).withSlot(0);


    
    //constructors

    rotationMotor = new SparkMax(KrotationMotorID, MotorType.kBrushless);
    hoodMotor = new TalonFX(KhoodMotorID);
    flywheelMotor = new TalonFX(KflywheelMotorID);
    flywheelMotor.getConfigurator().apply(flywheelConfig);

    turretRotationCANcoder = new CANcoder(KturretRotationCANcoderID, TunerConstants.kCANBus);
    hoodPitchCANcoder = new CANcoder(KhoodPitchCANcoderID);
    rotationCANcoderConfig = new CANcoderConfiguration();
    rotationCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    rotationCANcoderConfig.MagnetSensor.MagnetOffset = 0;
    
    turretRotationCANcoder.getConfigurator().apply(rotationCANcoderConfig);

    turretOffset = turretRotationCANcoder.getPosition().getValueAsDouble();



    rotationMotorPID = new PIDController(KrotationMotorkP, KrotationMotorkI, KrotationMotorkD);
    hoodMotorPID = new PIDController(KhoodMotorkP, KhoodMotorkI, KhoodMotorkP);

    rotationMotorPID.enableContinuousInput(-1.0, 1.0);
    hoodMotorPID.enableContinuousInput(-1.0, 1.0);

    leftLimSwitch = new DigitalInput(KleftLimSwitchID);
    rightLimSwitch = new DigitalInput(KrightLimSwitchID);


  }

  public boolean getLeftLimitSwitchVal(){
    return !leftLimSwitch.get();
  }

  public boolean getRightLimitSwitchVal(){
    return !rightLimSwitch.get();
  }

  // ==================== MOTOR ROTATIONS ====================


  // ==================== MOTOR ROTATIONS ====================
  // Maybe put in periodic?
  // Also  CanRotatedDegrees/55 = the current turret degree

  
 


  public void rotateRotationMotor(double power) { //rotates the main rotation motor of the turret
    //Make sure motor doesn't power when turret is outside of limits (0-270)
    if (getTurretRotationDegree() >= KrotationMotorRightLim || getTurretRotationDegree() <= KrotationMotorLeftLim)  {

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

  public void resetRotationDegree(double deg) {
    turretRotationCANcoder.setPosition((deg/360) * kturretToCancoderRatio);
  }

  //==================== FLYWHEEL ====================


  public void setFlyWheelVelocity(double velocity) {
    flywheelMotor.setControl(flywheelMotorRequest.withVelocity(velocity).withFeedForward(0.5));
  }

  public double getFlywheelMotorVelocity() {
    return flywheelMotor.getVelocity().getValueAsDouble();
  }

  // ==================== MOTOR DEGREES ====================
  public double getTurretRotationDegree() { 
    return ((turretRotationCANcoder.getPosition().getValueAsDouble() - KrotationMotorOffset) / kturretToCancoderRatio ) * 360; //converts it to a degree
  }

 
  public double getTurretRotationRaw(){
    return  turretRotationCANcoder.getPosition().getValueAsDouble();
  }

  public double getHoodDegree() {
    return (hoodPitchCANcoder.getAbsolutePosition().getValueAsDouble() - KhoodMotorOffset) * 360;
  }


  // ==================== MOVE TO FUNCTIONS ====================

  public double getRotationMotorPower(){
    return rotationMotor.get();
  }



  public void rotationMoveToPosition(double degrees) {
    double rotationDegree = getTurretRotationDegree();
    if (!(rotationDegree >= KrotationMotorRightLim && rotationDegree <= KrotationMotorLeftLim)) {
      rotateRotationMotor(rotationMotorPID.calculate(getTurretRotationDegree(), degrees));
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
    SmartDashboard.putNumber("rotation of turret ", getTurretRotationDegree());
    SmartDashboard.putNumber("rotation of turret raw",turretRotationCANcoder.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Left lim switch", getLeftLimitSwitchVal());
    SmartDashboard.putBoolean("Right lim switch", getRightLimitSwitchVal());

    if (getLeftLimitSwitchVal()) {
      resetRotationDegree(KrotationMotorLeftMagnetRot);   
    }
    else if (getRightLimitSwitchVal()) {
      resetRotationDegree(KrotationMotorRightMagnetRot);   

    }
  }
}
