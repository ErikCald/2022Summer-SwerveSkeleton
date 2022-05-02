// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    /**
     * Constructs a SwerveModule.
     */
    public SwerveModule() {
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getSteeringAngle());
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getSteeringAngle());
        Rotation2d angle = ContinousPIDSparkMax.calculate(state.angle, getSteeringAngle());
        double velocity = state.speedMetersPerSecond;
        
        // Convert the angle to SparkMax units (Revolutions) and pass to position PID.

        // Convert the velocity to SparkMax units (RPM) and pass to velocity PID.
    }

    public double getVelocity() {
        return 0.0;
    }

    public Rotation2d getSteeringAngle() {
        return Rotation2d.fromDegrees(0);
    }

    public void updateSteeringFromLamprey() {

    }

    public void stopMotors() {
    }

}
