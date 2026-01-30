package frc.robot.subsystems;


import com.revrobotics.spark.SparkFlex;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import static frc.robot.Constants.intakeConstants.*;





public class Intake extends SubsystemBase{
    private SparkFlex intakeMotorReel;
    private TalonFX armMotor;

  public Intake() {
      intakeMotorReel = new SparkFlex(KIntakeMotorReelId, MotorType.kBrushless);
      armMotor = new TalonFX(KArmMotorID);
    }

    //Reel
    public void reelIntake(double power) {
      intakeMotorReel.set(power);
    }

    public void reelArm(double power) {
      armMotor.set(power);
    }

    public void intakReelStop() {
      intakeMotorReel.set(0);
    }

    public void armReelStop() {
      armMotor.set(0);
    }



    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}