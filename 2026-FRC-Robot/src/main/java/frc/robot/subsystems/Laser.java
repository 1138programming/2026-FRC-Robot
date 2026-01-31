// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LaserConstants.*;

public class Laser extends SubsystemBase {
  DigitalOutput laser;
  /** Creates a new Laser. */
  public Laser() {
    laser = new DigitalOutput(kLaserDIOPort);
  }

  public void turnOn(){
    laser.set(true);
  }

  public void turnOff(){
    laser.set(false);
  }

  public void toggleLaser(){
    boolean laserVal = laser.get();
    laser.set(!laserVal);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
