package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.SpindexerConstants.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase{
    TalonFX rotationMotor;
    SparkMax indexerMotor;

    public Spindexer(){
        rotationMotor = new TalonFX(KRotationMotorID);
        indexerMotor = new SparkMax(KIndexMotorID, MotorType.kBrushless);
    }


    public void setRotationMotorPower(double power){
        rotationMotor.set(power);
    }


    public void setIndexerMotorPower(double power){
        indexerMotor.set(power);
    }

    public void stopRotationMotor(){
        rotationMotor.set(0);
    }
    
    public void stopIndexerMotor(){
        indexerMotor.set(0);
    }
    

    @Override
    public void periodic(){
    }
}
