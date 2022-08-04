// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  WPI_TalonFX front_left = new WPI_TalonFX(DriveConstants.kLeftMotor1Port);
  WPI_TalonFX back_left = new WPI_TalonFX(DriveConstants.kLeftMotor2Port);
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(
          front_left,
          back_left);

  // The motors on the right side of the drive.
  WPI_TalonFX front_right = new WPI_TalonFX(DriveConstants.kRightMotor1Port);
  WPI_TalonFX back_right = new WPI_TalonFX(DriveConstants.kRightMotor2Port);
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(
          front_right,
          back_right);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);


  // The gyro sensor (Pigeon 2)
  private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(DriveConstants.kPigeonPort);
  private final Gyro m_gyro = pigeon;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotors.setInverted(true);

    // Sets up the encoders
    front_left.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
    back_left.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
    front_right.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
    back_right.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(), 
        getLeftEncoderDistance(), 
        getRightEncoderDistance());
  }

  public double getLeftEncoderDistance() {
    return -((front_left.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse) + (back_left.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse)) / 2.0;
  }

  public double getRightEncoderDistance() {
    return ((front_right.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse) + (back_right.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse)) / 2.0;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds((-front_left.getSelectedSensorVelocity() - back_left.getSelectedSensorVelocity()) / 2.0, 
    (front_right.getSelectedSensorVelocity() + back_right.getSelectedSensorVelocity()) / 2.0);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
   front_left.setSelectedSensorPosition(0.0);
   back_left.setSelectedSensorPosition(0.0);
   front_right.setSelectedSensorPosition(0.0);
   back_right.setSelectedSensorPosition(0.0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance()) + (getRightEncoderDistance()) / 2.0;
  }

  /** CAN'T RETURN AN ENCODER BECAUSE FALCONS DON'T LET YOU
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  /*public Encoder getLeftEncoder() {
    return null;
  }*/

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  /*public Encoder getRightEncoder() {
    return null;
  }*/

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
