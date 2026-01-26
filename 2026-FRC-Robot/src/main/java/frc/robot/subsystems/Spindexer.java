package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.Constants.SpindexerConstants.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase{
    TalonFX rotationMotor;
    TalonFX indexerMotor;

    DigitalInput leftLimSwitch;
    DigitalInput rightLimSwitch;

    public Spindexer(){
        rotationMotor = new TalonFX(KRotationMotorID);
        indexerMotor = new TalonFX(KIndexMotorID);

        leftLimSwitch = new DigitalInput(KleftLimSwitchID);
        rightLimSwitch = new DigitalInput(KrightLimSwitchID);
    }

    public boolean getLeftLimTriggered(){
        return leftLimSwitch.get();
    }

    public boolean getRightLimTriggered(){
        return rightLimSwitch.get();
    }

    public void setRotationMotorPower(double power){
        rotationMotor.set(power);
    }

    public void setIndexerPower(double power){
        if (getLeftLimTriggered() && getRightLimTriggered()){
            indexerMotor.set(power);
            return;
        }
    }

    public void setRotationMotor(double power){
        rotationMotor.set(power);
    }
    

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("left lim", getLeftLimTriggered());
        SmartDashboard.putBoolean("right lim", getRightLimTriggered());
    }
}
