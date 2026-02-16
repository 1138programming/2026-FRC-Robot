// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Turret extends SubsystemBase {

  //defining
  private SparkMax rotationMotor;

  private TalonFX hoodMotor;
  // private TalonFX flywheelMotor;

  private SparkFlex flywheelMotor;

  private CANcoder turretRotationCANcoder;
  private CANcoder hoodPitchCANcoder;
  private CANcoderConfiguration rotationCANcoderConfig; 
  private PIDController rotationMotorPID;
  private SimpleMotorFeedforward rotationMotorFeedforward;
  private PIDController hoodMotorPID;
  private PIDController flywheelMotorPID;
  private final VelocityVoltage flywheelMotorRequest;
  private DigitalInput leftLimSwitch;
  private DigitalInput rightLimSwitch;

  private boolean updaterotleft;
  private boolean updaterotright;


  private double turretOffset;
  
  /** Creates a new Turret. */
  public Turret() {
    
    // constructors
    hoodMotor = new TalonFX(KhoodMotorID);
    flywheelMotor = new SparkFlex(19,MotorType.kBrushless);

    // Flywheel PID
    var flywheelConfig = new Slot0Configs();
    flywheelConfig.kP = KflywheelMotorP;
    flywheelConfig.kI = KflywheelMotorI;
    flywheelConfig.kD = KflywheelMotorD;
    flywheelMotorRequest = new VelocityVoltage(0).withSlot(0);


    
    //constructors

    rotationMotor = new SparkMax(KrotationMotorID, MotorType.kBrushless);
    hoodMotor = new TalonFX(KhoodMotorID);
    // flywheelMotor = new TalonFX(KflywheelMotorID);
    // flywheelMotor.getConfigurator().apply(flywheelConfig);

    turretRotationCANcoder = new CANcoder(KturretRotationCANcoderID, TunerConstants.kCANBus);
    hoodPitchCANcoder = new CANcoder(KhoodPitchCANcoderID);
    rotationCANcoderConfig = new CANcoderConfiguration();
    rotationCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    rotationCANcoderConfig.MagnetSensor.MagnetOffset = 0;
    
    turretRotationCANcoder.getConfigurator().apply(rotationCANcoderConfig);

    turretOffset = turretRotationCANcoder.getPosition().getValueAsDouble();

    


    SmartDashboard.putNumber("turret rot kp", KrotationMotorkP);
    SmartDashboard.putNumber("turret rot ki", KrotationMotorkI);
    SmartDashboard.putNumber("turret rot kd", KrotationMotorkD);

    SmartDashboard.putNumber("turret rot ks", KrotationMotorkS);
    SmartDashboard.putNumber("turret rot kv", KrotationMotorkV);

    SmartDashboard.putNumber("turret left magnet deg", KrotationMotorLeftMagnetRot);
    SmartDashboard.putNumber("turret right magnet deg", KrotationMotorRightMagnetRot);


    rotationMotorPID = new PIDController(KrotationMotorkP, KrotationMotorkI, KrotationMotorkD);
    rotationMotorFeedforward = new SimpleMotorFeedforward(KrotationMotorkS, KrotationMotorkV);
    hoodMotorPID = new PIDController(KhoodMotorkP, KhoodMotorkI, KhoodMotorkD);

    rotationMotorPID.disableContinuousInput();
    rotationMotorPID.setIZone(KrotationMotorkIzone);
    rotationMotorPID.setTolerance(Kturretsetpointoffset);
   
    hoodMotorPID.disableContinuousInput();

    leftLimSwitch = new DigitalInput(KleftLimSwitchID);
    rightLimSwitch = new DigitalInput(KrightLimSwitchID);

    updaterotleft =false;
    updaterotright = false;

  }

  // =================== BOUNDS ===================

  public boolean getLeftLimitSwitchVal(){
    return !leftLimSwitch.get(); //trust me bro its '!'
  }

  public boolean getRightLimitSwitchVal(){
    return !rightLimSwitch.get();
  }

  public boolean softStopLeft() {
    return getLeftLimitSwitchVal();
  }

  public boolean softStopRight() {
    return getRightLimitSwitchVal();
  }

  public boolean withinHardBounds(double angle) {
    return (angle > KrotationMotorRightLim || angle < KrotationMotorLeftLim);
  }
  public boolean withinBounds(double angle) {
    return (angle > KrotationMotorRightMagnetRot || angle < KrotationMotorLeftMagnetRot);
  }

  public boolean hasLeftReset() {
    return updaterotleft;
  }

  public boolean hasRightReset() {
    return updaterotright;
  }

  // ==================== MOTOR ROTATIONS ====================
  
  public void rotateRotationMotor(double power) { //rotates the main rotation motor of the turret
    //Make sure motor doesn't power when turret is outside of limits (0-270)
    if (getTurretRotationDegree() >= KrotationMotorRightMagnetRot  && power >0)  {
      rotationMotor.set(Math.min(power,0.4));
      return;
    }
    else if ( getTurretRotationDegree() <= KrotationMotorLeftMagnetRot && power <0) {
      rotationMotor.set(Math.max(power,-0.4));
      return;
     }

    rotationMotor.set(power);
  }

  public void rotateRotationMotorAtVelocity(double Velocity) { //rotates the main rotation motor of the turret
    //Make sure motor doesn't power when turret is outside of limits (0-270)
    if (getTurretRotationDegree() >= KrotationMotorRightMagnetRot)  {
      Velocity = Math.min(Velocity,10);
      
    }
    else if (getTurretRotationDegree() <= KrotationMotorLeftMagnetRot) {
      Velocity = Math.max(Velocity,-10);
    }
    double out = rotationMotorFeedforward.calculateWithVelocities(getRotationVelocityDegPerSecond(),Velocity);
    //rotationMotorFeedforward.calculateWithVelocities(getRotationVelocityDegPerSecond(),Velocity);
    SmartDashboard.putNumber("power out", out);
    SmartDashboard.putNumber("velocity out", Velocity);

    rotateRotationMotor(out);
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
    turretRotationCANcoder.setPosition(((deg/360) * kturretToCancoderRatio)- KrotationMotorOffset);
  }

  //==================== FLYWHEEL ====================

  // public void setFlyWheelVelocity(double velocity) {
  //   flywheelMotor.setControl(flywheelMotorRequest.withVelocity(velocity).withFeedForward(0.5));
  // }

  // public double getFlywheelMotorVelocity() {
  //   return flywheelMotor.getVelocity().getValueAsDouble();
  // }

  public void setFlyWheelVelocity(double velocity) {
    flywheelMotor.set(velocity);
  }

  public double getFlywheelMotorVelocity() {
    return flywheelMotor.getEncoder().getVelocity();
  }

  // ==================== MOTOR DEGREES ====================
  public double getTurretRotationDegree() { 
    return ((turretRotationCANcoder.getPosition().getValueAsDouble() - KrotationMotorOffset) / kturretToCancoderRatio ) * 360 ; //converts it to a degree
  }

    public double getRotationVelocityDegPerSecond() {
    return (turretRotationCANcoder.getVelocity().getValueAsDouble()/ kturretToCancoderRatio) * 360;
  }

  public double getTurretRotationRaw(){
    return  turretRotationCANcoder.getPosition().getValueAsDouble();
  }

  public double getHoodDegree() {
    return (hoodPitchCANcoder.getAbsolutePosition().getValueAsDouble() - KhoodMotorOffset) * 360;
  }


  // ==================== STOP FUNCTIONS ========================

  public void stopTurretRotation() {
    rotationMotor.set(0.0);
  }

  public void stopFlywheel() {
    flywheelMotor.set(0.0);
  }

  public void stopHood() {
    hoodMotor.set(0.0);
  }

  public void stopAll() {
    stopTurretRotation();
    stopFlywheel();
    stopHood();
  }
  
  // ==================== MOVE TO FUNCTIONS ====================

  public double getRotationMotorPower(){
    return rotationMotor.get();
  }

  public boolean rotationMoveToPosition(double degrees, double feedforward) {
    double power = rotationMotorPID.calculate(getTurretRotationDegree(), degrees) * KrotationMotorCoefficient;
    feedforward = rotationMotorFeedforward.calculate(feedforward);
    power = Math.min(power,KrotationMotorMaxVelocity);
    power = Math.max(power,-KrotationMotorMaxVelocity);
    // if (power > 0 ) {
    //   power = Math.max(power,KrotationMotorMinVelocity);
    // }
    // else if (power < 0 ) {
    //   power = Math.min(power,-KrotationMotorMinVelocity);
    // }

    if (rotationMotorPID.atSetpoint()) {
      rotateRotationMotor(0);
    }
    else{
      rotateRotationMotor(power + feedforward);
    }
    SmartDashboard.putBoolean("pid at setpoint",rotationMotorPID.atSetpoint());
    SmartDashboard.putNumber("pid error",rotationMotorPID.getError());
    Logger.recordOutput("pid error",rotationMotorPID.getError());

    // SmartDashboard.putNumber("feedforward",rotationMotorFeedforward.calculate(feedforward));
    return rotationMotorPID.atSetpoint();
  }

    /**
   *  Resets the previous error and integral terms of the turret PID
   */
  public void turretResetrotationmotorpid() {
    rotationMotorPID.reset();
  }

  /**
   * 
   * @return wether the turret rotation is with the setpoint +/- the tolerance of the PID
   */
  public boolean turretRotationmotorpidatsetpoint() {
    return rotationMotorPID.atSetpoint();
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

    // SmartDashboard.putNumber("flywheel speed", getFlywheelMotorVelocity());
    SmartDashboard.putNumber("rotation speed", getRotationVelocityDegPerSecond());
    Logger.recordOutput("rotation speed", getRotationVelocityDegPerSecond());



    
    SmartDashboard.putBoolean("Left lim switch", getLeftLimitSwitchVal());
    SmartDashboard.putBoolean("Right lim switch", getRightLimitSwitchVal());

    SmartDashboard.putBoolean("reset left", updaterotleft);
    SmartDashboard.putBoolean("reset right", updaterotright);

    rotationMotorPID.setP(SmartDashboard.getNumber("turret rot kp", KrotationMotorkP));
    rotationMotorPID.setI(SmartDashboard.getNumber("turret rot ki", KrotationMotorkI));
    rotationMotorPID.setD(SmartDashboard.getNumber("turret rot kd", KrotationMotorkD));

    rotationMotorFeedforward.setKs(SmartDashboard.getNumber("turret rot ks", KrotationMotorkS));
    rotationMotorFeedforward.setKv(SmartDashboard.getNumber("turret rot kv", KrotationMotorkV));
    
    if (getLeftLimitSwitchVal() && !updaterotleft) {
      resetRotationDegree(SmartDashboard.getNumber("turret left magnet deg", KrotationMotorLeftMagnetRot)); 
      updaterotleft = true; 
      updaterotright = false;
      // System.out.println("reset left");
    

    }
    else if (getRightLimitSwitchVal() && !updaterotright) {
      resetRotationDegree(SmartDashboard.getNumber("turret right magnet deg", KrotationMotorRightMagnetRot));   
      updaterotright = true; 
      updaterotleft = false; 
      // System.out.println("reset right");
    }

    // if (!getLeftLimitSwitchVal() || !getRightLimitSwitchVal()) {
    //   updaterot =false;
    // }
  }
}
