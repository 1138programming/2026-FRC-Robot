package frc.robot.subsystems;


import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;


import static frc.robot.Constants.*;





public class Intake extends SubsystemBase{
      private SparkFlex intakeMotorReel;



public Intake(){
    intakeMotorReel = new SparkFlex(KIntakeMotorReelId, MotorType.kBrushless);
  }

  //Reel
  public void reelIntake(double speed){
    intakeMotorReel.set(speed);
  }

  public void intakReelStop(){
    intakeMotorReel.set(0);
  }

//Extend



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}