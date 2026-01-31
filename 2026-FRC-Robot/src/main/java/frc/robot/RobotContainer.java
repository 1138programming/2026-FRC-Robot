
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.TurretConstants.*;

//subsystem
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;


import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.toggleLaser;
import frc.robot.commands.TurretCommands.TurretRotate;
import frc.robot.commands.TurretCommands.TurretSetRotation;
import frc.robot.subsystems.Laser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.Constants.LimelightConstants;
//drive
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TurretTracking;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;  
  public final Laser m_Laser;
  public final Turret m_Turret;
  public final Limelight limelight;


  // Comands
  public final TurretTracking m_Turret_Tracking;
  public final TurretRotate m_Turret_Rotate_Forward;
  public final TurretSetRotation turretresetRot; 
  public final TurretRotate m_Turret_Rotate_Backward;
  public final toggleLaser lasertoggle;



  // Comands

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

   public static Joystick logitech;
  public static Joystick compStreamDeck;
  public static Joystick testStreamDeck;
  public static Joystick autonTestStreamDeck;
  // public final CommandXboxController joystick = new CommandXboxController(1);
  public JoystickButton logitechBtnX,
      logitechBtnA,
      logitechBtnB,
      logitechBtnY,
      logitechBtnLB,
      logitechBtnRB,
      logitechBtnLT,
      logitechBtnRT,
      logitechBtnBack,
      logitechBtnStart; // Logitech Button
  public JoystickButton compStreamDeck1,
      compStreamDeck2,
      compStreamDeck3,
      compStreamDeck4,
      compStreamDeck5,
      compStreamDeck6,
      compStreamDeck7,
      compStreamDeck8,
      compStreamDeck9,
      compStreamDeck10,
      compStreamDeck11,
      compStreamDeck12,
      compStreamDeck13,
      compStreamDeck14,
      compStreamDeck15,
      compStreamDeck16,
      compStreamDeck17,
      compStreamDeck18,
      compStreamDeck19;

  // Top Left SD = 1, numbered from left to right
  public JoystickButton testStreamDeck1,
      testStreamDeck2,
      testStreamDeck3,
      testStreamDeck4,
      testStreamDeck5,
      testStreamDeck6,
      testStreamDeck7,
      testStreamDeck8,
      testStreamDeck9, // Vjoy 2
      testStreamDeck10,
      testStreamDeck11,
      testStreamDeck12,
      testStreamDeck13,
      testStreamDeck14,
      testStreamDeck15;
  public JoystickButton autonTestStreamDeck1,
      autonTestStreamDeck2,
      autonTestStreamDeck3,
      autonTestStreamDeck4,
      autonTestStreamDeck5,
      autonTestStreamDeck6,
      autonTestStreamDeck7,
      autonTestStreamDeck8,
      autonTestStreamDeck9, // Vjoy 2
      autonTestStreamDeck10,
      autonTestStreamDeck11,
      autonTestStreamDeck12,
      autonTestStreamDeck13,
      autonTestStreamDeck14,
      autonTestStreamDeck15;

  private double baseSpeed = KBaseNormalMode;
 

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_Turret = new Turret();
    m_Turret_Tracking = new TurretTracking(m_Turret);
    m_Turret_Rotate_Forward = new TurretRotate(m_Turret, 0.15);
    m_Turret_Rotate_Backward = new TurretRotate(m_Turret, -0.15);
    turretresetRot = new TurretSetRotation(m_Turret, 0);

    m_Laser = new Laser();
    lasertoggle = new toggleLaser(m_Laser);
    limelight = new Limelight(LimelightConstants.kLimelightName);
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder

        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                limelight);

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight),
                limelight);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                limelight);
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    

    logitech = new Joystick(KLogitechPort); // Logitech Dual Action
    compStreamDeck = new Joystick(KCompStreamDeckPort); // Stream Deck + vjoy
    testStreamDeck = new Joystick(KTestingStreamDeckPort); // Stream Deck + vjoy
    autonTestStreamDeck = new Joystick(KAutonTestingStreamDeckPort); // Stream Deck + vjoy

    // Logitch Buttons
    logitechBtnX = new JoystickButton(logitech, KLogitechButtonX);
    logitechBtnA = new JoystickButton(logitech, KLogitechButtonA);
    logitechBtnB = new JoystickButton(logitech, KLogitechButtonB);
    logitechBtnY = new JoystickButton(logitech, KLogitechButtonY);
    logitechBtnLB = new JoystickButton(logitech, KLogitechLeftBumper);
    logitechBtnRB = new JoystickButton(logitech, KLogitechRightBumper);
    logitechBtnLT = new JoystickButton(logitech, KLogitechLeftTrigger);
    logitechBtnRT = new JoystickButton(logitech, KLogitechRightTrigger);
    logitechBtnBack = new JoystickButton(logitech, KLogitechBtnBack);
    logitechBtnStart = new JoystickButton(logitech, KLogitechRightStart);

    // Streamdeck Pages used in match
    compStreamDeck1 = new JoystickButton(compStreamDeck, 1);
    compStreamDeck2 = new JoystickButton(compStreamDeck, 2);
    compStreamDeck3 = new JoystickButton(compStreamDeck, 3);
    compStreamDeck4 = new JoystickButton(compStreamDeck, 4);
    compStreamDeck5 = new JoystickButton(compStreamDeck, 5);
    compStreamDeck6 = new JoystickButton(compStreamDeck, 6);
    compStreamDeck7 = new JoystickButton(compStreamDeck, 7);
    compStreamDeck8 = new JoystickButton(compStreamDeck, 8);
    compStreamDeck9 = new JoystickButton(compStreamDeck, 9);
    compStreamDeck10 = new JoystickButton(compStreamDeck, 10);
    compStreamDeck11 = new JoystickButton(compStreamDeck, 11);
    compStreamDeck12 = new JoystickButton(compStreamDeck, 12);
    compStreamDeck13 = new JoystickButton(compStreamDeck, 13);
    compStreamDeck14 = new JoystickButton(compStreamDeck, 14);
    compStreamDeck15 = new JoystickButton(compStreamDeck, 15);
    compStreamDeck16 = new JoystickButton(compStreamDeck, 16);
    compStreamDeck17 = new JoystickButton(compStreamDeck, 17);
    compStreamDeck18 = new JoystickButton(compStreamDeck, 18);
    compStreamDeck19 = new JoystickButton(compStreamDeck, 19);

    // Streamdeck Pages used for testing
    testStreamDeck1 = new JoystickButton(testStreamDeck, 1);
    testStreamDeck2 = new JoystickButton(testStreamDeck, 2);
    testStreamDeck3 = new JoystickButton(testStreamDeck, 3);
    testStreamDeck4 = new JoystickButton(testStreamDeck, 4);
    testStreamDeck5 = new JoystickButton(testStreamDeck, 5);
    testStreamDeck6 = new JoystickButton(testStreamDeck, 6);
    testStreamDeck7 = new JoystickButton(testStreamDeck, 7);
    testStreamDeck8 = new JoystickButton(testStreamDeck, 8);
    testStreamDeck9 = new JoystickButton(testStreamDeck, 9);
    testStreamDeck10 = new JoystickButton(testStreamDeck, 10);
    testStreamDeck11 = new JoystickButton(testStreamDeck, 11);
    testStreamDeck12 = new JoystickButton(testStreamDeck, 12);
    testStreamDeck13 = new JoystickButton(testStreamDeck, 13);
    testStreamDeck14 = new JoystickButton(testStreamDeck, 14);
    testStreamDeck15 = new JoystickButton(testStreamDeck, 15);

    autonTestStreamDeck1 = new JoystickButton(testStreamDeck, 1);
    autonTestStreamDeck2 = new JoystickButton(testStreamDeck, 2);
    autonTestStreamDeck3 = new JoystickButton(testStreamDeck, 3);
    autonTestStreamDeck4 = new JoystickButton(testStreamDeck, 4);
    autonTestStreamDeck5 = new JoystickButton(testStreamDeck, 5);
    autonTestStreamDeck6 = new JoystickButton(testStreamDeck, 6);
    autonTestStreamDeck7 = new JoystickButton(testStreamDeck, 7);
    autonTestStreamDeck8 = new JoystickButton(testStreamDeck, 9);
    autonTestStreamDeck9 = new JoystickButton(testStreamDeck, 9);
    autonTestStreamDeck10 = new JoystickButton(testStreamDeck, 10);
    autonTestStreamDeck11 = new JoystickButton(testStreamDeck, 11);
    autonTestStreamDeck12 = new JoystickButton(testStreamDeck, 12);
    autonTestStreamDeck13 = new JoystickButton(testStreamDeck, 13);
    autonTestStreamDeck14 = new JoystickButton(testStreamDeck, 14);
    autonTestStreamDeck15 = new JoystickButton(testStreamDeck, 15);



    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> getLogiLeftYAxis() * 0.75,
            () -> getLogiLeftXAxis() * 0.75,
            () -> getLogiRightXAxis() * 0.75));

    // Lock to 0° when A button is held
    // logitechBtnA
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> getLogiLeftYAxis(),
    //             () -> getLogiLeftXAxis(),
    //             () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    // logitechBtnX.onTrue(Commands.runOnce(drive::stopWithX, drive));
    logitechBtnB.whileTrue(m_Turret_Rotate_Forward);

    logitechBtnA.whileTrue(m_Turret_Rotate_Backward);

    logitechBtnRB.onTrue(turretresetRot);


    // Reset gyro to 0° when B button is pressed
    logitechBtnY
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
  }



  
  public double getLogiRightYAxis() {
    final double Y = logitech.getRawAxis(KRightYAxis);
    SmartDashboard.putNumber("getLogiRightYAxis", -Y);
    if (Y > KDeadZone || Y < -KDeadZone) return -Y;
    else return 0;
  }

  public double getLogiLeftYAxis() {
    final double Y = logitech.getY();
    SmartDashboard.putNumber("getLogiLeftYAxis", -Y);
    if (Y > KDeadZone || Y < -KDeadZone) return -Y;
    else return 0;
  }

  public double getLogiRightXAxis() {
    double X = logitech.getZ();
    SmartDashboard.putNumber("getLogiRightXAxis", -X);
    if (X > KDeadZone || X < -KDeadZone) {
      return -X;
    } else {
      return 0;
    }
  }

  public double getLogiLeftXAxis() {
    double X = logitech.getX();
    SmartDashboard.putNumber("getLogiLeftXAxis", -X);
    if (X > KDeadZone || X < -KDeadZone) {

      return -X;
    } else {
      return 0;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
