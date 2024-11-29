// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.enums.DrivetrainState;
import frc.robot.enums.IntakeState;
import frc.robot.enums.JukeboxState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SDSDrivetrain;
import frc.robot.subsystems.DrivetrainSim;
import frc.robot.subsystems.IDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Jukebox;
import frc.robot.subsystems.Vision;
import frc.robot.utilities.GeometryUtil;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  private final CommandXboxController _driverController;
  private final SDSDrivetrain _drivetrain;

  private final Vision _vision;
  private final Intake _intake;
  private final Jukebox _jukebox;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    _driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    _vision = new Vision(
      Constants.VisionConstants.kTargeterLimelightName,
      Constants.VisionConstants.kLimelightNames,
      Constants.VisionConstants.kPhotonCameraNames
    );

    _intake = new Intake();
    _drivetrain = new SDSDrivetrain(_driverController, _vision);
    _jukebox = new Jukebox(_drivetrain);
    // m_drivetrain = new DrivetrainSim(m_driverController);
    NamedCommands.registerCommand("Target Speaker", _drivetrain.targetSpeaker(GeometryUtil::isRedAlliance));
    NamedCommands.registerCommand("Initialize Auto", _drivetrain.setWantedState(DrivetrainState.TRAJECTORY_FOLLOW));

    // Configure the trigger bindings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  
  }

  private void configureBindings() {
    _drivetrain.setDefaultCommand(_drivetrain.fieldDrive(0, 0, 0));
    if (_jukebox.hasNote()) {
      _driverController.leftTrigger().onTrue(new ParallelCommandGroup(_drivetrain.setWantedState(DrivetrainState.TARGET_FOLLOW), _jukebox.setWantedState(JukeboxState.PREP)))
                                    .onFalse(new ParallelCommandGroup(_drivetrain.setWantedState(DrivetrainState.OPEN_LOOP), _jukebox.setWantedState(JukeboxState.SCORE)));
    }
    // _driverController.leftBumper().onTrue(_drivetrain.targetZone(GeometryUtil::isRedAlliance))
    //                                 .onFalse(_drivetrain.setWantedState(DrivetrainState.OPEN_LOOP));
    _driverController.a().onTrue(_drivetrain.setWantedState(DrivetrainState.NOTE_FOLLOW))
                          .onFalse(_drivetrain.setWantedState(DrivetrainState.OPEN_LOOP));
    
    _driverController.y().onTrue(new ParallelCommandGroup(new InstantCommand(()-> _drivetrain.setTargetSpeaker(GeometryUtil::isRedAlliance)), new InstantCommand(() -> _jukebox.setTargetSpeaker())));

    _driverController.b().onTrue(new ParallelCommandGroup(new InstantCommand(()-> _drivetrain.setTargetAmp(GeometryUtil::isRedAlliance)), new InstantCommand(() -> _jukebox.setTargetAmp())));

    _driverController.x().onTrue(new ParallelCommandGroup(new InstantCommand (()-> _drivetrain.setTargetZone(GeometryUtil::isRedAlliance)), new InstantCommand(() -> _jukebox.setTargetZone())));

    _driverController.rightBumper().onTrue(_intake.setWantedState(IntakeState.EJECT));    

    _driverController.start().onTrue(_drivetrain.resetGyro());
    new Trigger(DriverStation::isTeleopEnabled).onTrue(_drivetrain.setWantedState(DrivetrainState.OPEN_LOOP));
    
    new Trigger(_jukebox::hasNote).onFalse(_intake.setWantedState(IntakeState.INTAKE)).onTrue(_intake.setWantedState(IntakeState.PASSIVE_EJECT));

    new Trigger(_jukebox::doneScoring).onTrue(new ParallelCommandGroup(_drivetrain.setWantedState(DrivetrainState.OPEN_LOOP), _jukebox.setWantedState(JukeboxState.IDLE)));

    //new Trigger(DriverStation::isAutonomousEnabled).onTrue(_drivetrain.setWantedState(DrivetrainState.TRAJECTORY_FOLLOW));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
