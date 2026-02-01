// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Turret;
import frc.robot.subsystems.ShooterLogic;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAim extends Command {
  /** Creates a new AutoAim. */

  private Turret turret;
  private ShooterLogic shooterLogic;

  public AutoAim(Turret turret, ShooterLogic shooterLogic) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.shooterLogic = shooterLogic;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double[] logicData = shooterLogic.calculateShotChanges();
    turret.setFlyWheelVelocity(Math.toDegrees(logicData[1]));
    turret.hoodMoveToPosition(Math.toDegrees(logicData[2]));
    turret.rotationMoveToPosition(Math.toDegrees(logicData[0]));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopFlywheel(); //only stop flywheel cause other methods stop on their own
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
