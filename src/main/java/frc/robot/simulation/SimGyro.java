package frc.robot.simulation;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Basic simulation of a gyro, will just hold its current state and not use any
 * hardware
 */
public class SimGyro {
    private Rotation2d currentRotation = new Rotation2d();

    public Rotation2d getRotation2d() {
        return currentRotation;
    }

    public void updateRotation(double angularVelRps) {
        currentRotation = currentRotation.plus(new Rotation2d(angularVelRps * 0.02));
    }
}