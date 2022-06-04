// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.config.Config;

public class SwerveModule {

    CANSparkMax driveMotor;
    CANSparkMax steeringMotor;

    SparkMaxPIDController drivePIDController;
    SparkMaxPIDController steeringPIDController;

    RelativeEncoder driveEncoder;
    RelativeEncoder steeringEncoder;

    /**
     * Constructs a SwerveModule.
     */
    public SwerveModule(int driveCANID, int steerCANID, boolean driveInverted, boolean steerInverted) {

        // Drive Motor
        driveMotor = new CANSparkMax(driveCANID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setIdleMode(IdleMode.kCoast);
        driveMotor.setInverted(driveInverted);
        driveMotor.setSmartCurrentLimit(Config.CURRENT_LIMIT_DRIVE);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setVelocityConversionFactor(Config.DRIVE_VEL_CONVERSION);

        drivePIDController = driveMotor.getPIDController();
        drivePIDController.setFF(Config.DRIVE_FF);
        drivePIDController.setP(Config.DRIVE_P);
        drivePIDController.setI(Config.DRIVE_I);
        drivePIDController.setD(Config.DRIVE_D);
        drivePIDController.setIZone(Config.DRIVE_IZONE);

        // Steering motor
        steeringMotor = new CANSparkMax(steerCANID, MotorType.kBrushless);
        steeringMotor.restoreFactoryDefaults();
        steeringMotor.setIdleMode(IdleMode.kCoast);
        steeringMotor.setInverted(steerInverted);
        steeringMotor.setSmartCurrentLimit(Config.CURRENT_LIMIT_STEERING);

        steeringEncoder = steeringMotor.getEncoder();
        steeringEncoder.setPositionConversionFactor(Config.STEERING_POS_CONVERSION);

        steeringPIDController = driveMotor.getPIDController();
        steeringPIDController.setFF(Config.STEERING_FF);
        steeringPIDController.setP(Config.STEERING_P);
        steeringPIDController.setI(Config.STEERING_I);
        steeringPIDController.setD(Config.STEERING_D);
        steeringPIDController.setIZone(Config.STEERING_IZONE);
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
        double velocity = state.speedMetersPerSecond;
        Rotation2d angle = ContinousPIDSparkMax.calculate(state.angle, measuredAngle);
        
        drivePIDController.setReference(velocity, ControlType.kVelocity);
        steeringPIDController.setReference(angle.getRadians(), ControlType.kPosition);
    }

    /**
     * Returns the velocity of the wheel in meters per second.
     * 
     * @return meters per second of the wheel
     */
    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Returns the angle the wheel is pointing in a Rotation2d.
     * 
     * @return angle of the wheel
     */
    public Rotation2d getSteeringAngle() {
        return new Rotation2d(steeringEncoder.getPosition());
    }

    /**
     * Gets a reading from the Lamprey and updates the SparkMax encoder (interal NEO encoder).
     * This is specific to Swerge. Other methods need to be written for other hardware.
     */
    public void updateSteeringFromLamprey() {
        
        // CODE: You can attempt this if you want but this will probably be done together in the 2nd or 3rd meeting.
        // CODE: Read value from Lamprey and set internal Neo encoder for the steering SparkMax (but need to add an offset first)

    }

    /**
     * Standard stop motors method for every subsystem.
     */
    public void stopMotors() {
        driveMotor.stopMotor();
        steeringMotor.stopMotor();
    }

}
