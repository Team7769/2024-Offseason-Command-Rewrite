// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoInitializeCommand;
import frc.robot.commands.TargetSpeakerCommand;
import frc.robot.commands.TargetZoneCommand;
import frc.robot.commands.TeleopInitializeCommand;
import frc.robot.enums.DrivetrainState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DrivetrainSim;
import frc.robot.subsystems.IDrivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final IDrivetrain m_drivetrain;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    m_drivetrain = new Drivetrain(m_driverController);
    //m_drivetrain = new DrivetrainSim(m_driverController);

    NamedCommands.registerCommand("Target Speaker", new TargetSpeakerCommand(m_drivetrain));
    NamedCommands.registerCommand("Initialize Auto", new AutoInitializeCommand(m_drivetrain));

    // Configure the trigger bindings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  }
  
  private void configureBindings() {
    m_driverController.leftTrigger().onTrue(new TargetSpeakerCommand(m_drivetrain))
                                    .onFalse(new InstantCommand(() -> m_drivetrain.setWantedState(DrivetrainState.OPEN_LOOP)));
    m_driverController.leftBumper().onTrue(new TargetZoneCommand(m_drivetrain))
                                    .onFalse(new InstantCommand(() -> m_drivetrain.setWantedState(DrivetrainState.OPEN_LOOP)));

    new Trigger(DriverStation::isTeleopEnabled).onTrue(new TeleopInitializeCommand(m_drivetrain));
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
