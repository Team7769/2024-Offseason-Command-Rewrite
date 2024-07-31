// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.enums.DrivetrainState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DrivetrainSim;
import frc.robot.subsystems.IDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.utilities.GeometryUtil;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

  private final CommandXboxController m_driverController;
  private final Drivetrain m_drivetrain;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    Vision vision = new Vision(
      Constants.VisionConstants.kTargeterLimelightName,
      Constants.VisionConstants.kLimelightNames,
      Constants.VisionConstants.kPhotonCameraNames
    );

    m_drivetrain = new Drivetrain(m_driverController, vision);
    // m_drivetrain = new DrivetrainSim(m_driverController);
    NamedCommands.registerCommand("Target Speaker", m_drivetrain.targetSpeaker(GeometryUtil::isRedAlliance));
    NamedCommands.registerCommand("Initialize Auto", m_drivetrain.setWantedState(DrivetrainState.TRAJECTORY_FOLLOW));

    // Configure the trigger bindings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void configureBindings() {
    m_drivetrain.setDefaultCommand(m_drivetrain.applyRequest(() -> m_drivetrain.idle));
    m_driverController.leftTrigger().onTrue(m_drivetrain.targetSpeaker(GeometryUtil::isRedAlliance))
                                    .onFalse(m_drivetrain.setWantedState(DrivetrainState.OPEN_LOOP));
    m_driverController.leftBumper().onTrue(m_drivetrain.targetZone(GeometryUtil::isRedAlliance))
                                    .onFalse(m_drivetrain.setWantedState(DrivetrainState.OPEN_LOOP));
    m_driverController.start().onTrue(m_drivetrain.resetGyro());
    new Trigger(DriverStation::isTeleopEnabled).onTrue(m_drivetrain.setWantedState(DrivetrainState.OPEN_LOOP));

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
