package frc.robot.subsystems;


import com.revrobotics.spark.SparkFlex;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import static frc.robot.Constants.intakeConstants.*;





public class Intake extends SubsystemBase{
    private SparkFlex intakeMotor;
    private TalonFX intakeDeployMotor;

  public Intake() {
      intakeMotor = new SparkFlex(KintakeMotorId, MotorType.kBrushless);
      intakeDeployMotor = new TalonFX(KintakeDeployMotorId);
    }

    public void setIntakePowwer(double power) {
      intakeMotor.set(power);
    }

    public void setIntakeDeployMotorPower(double power) {
      intakeDeployMotor.set(power);
    }

    public void stopIntakeMotor() {
      intakeMotor.set(0);
    }

    public void stopIntakeDeployMotor() {
      intakeDeployMotor.set(0);
    }



    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}