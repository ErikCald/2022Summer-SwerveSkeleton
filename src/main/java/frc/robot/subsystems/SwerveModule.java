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
        Rotation2d measuredAngle = getSteeringAngle();
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, measuredAngle);
        Rotation2d angle = ContinousPIDSparkMax.calculate(state.angle, measuredAngle);
        double velocity = state.speedMetersPerSecond;
        
        // Set the PositionConversionFactor ahead of time and then pass radians to position PID on steering SparkMax.

        // Set the VelocityConversionFactor ahead of time and then pass m/s to velocity PID on drive SparkMax.
    }

    public double getVelocity() {

        // Read encoder velocity from drive SparkMax, set the VelocityConversionFactor ahead of time so it's in unit of m/s.

        return 0.0;
    }

    public Rotation2d getSteeringAngle() {

        // Read encoder position from steering SparkMax, set the PositionConversionFactor ahead of time so it's in unit of radians.

        return new Rotation2d(0);
    }

    public void updateSteeringFromLamprey() {
        
        // Read value from Lamprey and set internal Neo encoder for the steering SparkMax (but need to add an offset first)

    }

    public void stopMotors() {

        // Call the stopMotors method (provided with all WPILib motor controller objects)

    }

}
