// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.*;

import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.ShooterLogic;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretAutoAim extends Command {

  Turret turret;
  ShooterLogic logic;
  // boolean atpos;

  /** Creates a new TurrentTracking. */
  public TurretAutoAim(Turret turret, ShooterLogic logic) {
    this.turret = turret;
    this.logic = logic;
    // atpos = false;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.turretResetrotationmotorpid();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // atpos = turret.rotationMoveToPosition(logic.relativeTurretAngletoPos(FieldConstants.HubConstants.red.kHubFieldPose2d) + TurretConstants.KturretFlywheelOffset); 
    logic.turretTrackPose(FieldConstants.HubConstants.red.kHubFieldPose2d);
  }

  // Called once the command ends or is interrupted. 67
  @Override
  public void end(boolean interrupted) {
    turret.rotateRotationMotor(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}