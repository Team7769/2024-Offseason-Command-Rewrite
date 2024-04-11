package frc.robot.simulation;

import edu.wpi.first.math.kinematics.*;

/**
 * Basic simulation of a swerve module, will just hold its current state and not
 * use any hardware
 */
public class SimSwerveModule {
    private SwerveModulePosition currentPosition = new SwerveModulePosition();
    private SwerveModuleState currentState = new SwerveModuleState();

    public SwerveModulePosition getPosition() {
        return currentPosition;
    }

    public SwerveModuleState getState() {
        return currentState;
    }

    public void setTargetState(SwerveModuleState targetState) {
        // Optimize the state
        currentState = SwerveModuleState.optimize(targetState, currentState.angle);

        currentPosition = new SwerveModulePosition(
                currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
    }
}