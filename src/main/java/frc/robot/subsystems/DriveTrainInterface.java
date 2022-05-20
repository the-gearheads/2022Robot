package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DriveTrainInterface extends Subsystem{
    /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds);
  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double rot);
  public void drive(ChassisSpeeds chassisSpeeds);

  public void debugDrive(double kS, double backwardkS, double kV, double backwardkV, double xSpeed, double rot);

  /** Updates the field-relative position. */
  public void updateOdometry();

  public void debugDrive(double xSpeed, double rot);

  public Pose2d getFieldPosition();

  public double getRotation();

  public double getRightPos();
  public double getLeftPos();

  public double getRightVelocity();
  public double getLeftVelocity();
  public void zeroEncoders();
  public void setFieldPos(Pose2d currentPos);

  public void setSpeeds(double leftSpeed, double rightSpeed);

  public void printValues();
  public void setRampRate(double val);
  public void setRampRate(boolean isTrue);
}
