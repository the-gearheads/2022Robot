// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Represents a differential drive style drivetrain. */
public class DriveTrainSS extends SubsystemBase{

  private final CANSparkMax rfMotor = new CANSparkMax(Constants.DriveTrain.RFMOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax rbMotor = new CANSparkMax(Constants.DriveTrain.RBMOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax lfMotor = new CANSparkMax(Constants.DriveTrain.LFMOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax lbMotor = new CANSparkMax(Constants.DriveTrain.LBMOTOR_ID, MotorType.kBrushless);

  private final RelativeEncoder rfEncoder = rfMotor.getEncoder();
  private final RelativeEncoder rbEncoder = rbMotor.getEncoder();
  private final RelativeEncoder lfEncoder = lfMotor.getEncoder();
  private final RelativeEncoder lbEncoder = lbMotor.getEncoder();

  private final MotorControllerGroup rightGroup =
      new MotorControllerGroup(lfMotor, lbMotor);
  private final MotorControllerGroup leftGroup =
      new MotorControllerGroup(rfMotor, rbMotor);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private final PIDController rightPIDController = new PIDController(
    Constants.DriveTrain.RIGHT_kP,
    Constants.DriveTrain.RIGHT_kI,
    Constants.DriveTrain.RIGHT_kD);
    private final PIDController leftPIDController = new PIDController(
      Constants.DriveTrain.LEFT_kP,
      Constants.DriveTrain.LEFT_kI,
      Constants.DriveTrain.LEFT_kD);

  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Constants.DriveTrain.TRACK_WIDTH);

  private final DifferentialDriveOdometry odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.DriveTrain.kS, Constants.DriveTrain.kV);

  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
   * gyro.
   */
  public DriveTrainSS() {
    gyro.reset();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightGroup.setInverted(true);

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    rfEncoder.setPositionConversionFactor(2*Math.PI*Constants.DriveTrain.WHEEL_RADIUS);
    rfEncoder.setVelocityConversionFactor(2*Math.PI*Constants.DriveTrain.WHEEL_RADIUS);
    rbEncoder.setPositionConversionFactor(2*Math.PI*Constants.DriveTrain.WHEEL_RADIUS);
    rbEncoder.setVelocityConversionFactor(2*Math.PI*Constants.DriveTrain.WHEEL_RADIUS);
    lfEncoder.setPositionConversionFactor(2*Math.PI*Constants.DriveTrain.WHEEL_RADIUS);
    lfEncoder.setVelocityConversionFactor(2*Math.PI*Constants.DriveTrain.WHEEL_RADIUS);
    lbEncoder.setPositionConversionFactor(2*Math.PI*Constants.DriveTrain.WHEEL_RADIUS);
    lbEncoder.setVelocityConversionFactor(2*Math.PI*Constants.DriveTrain.WHEEL_RADIUS);

    rfEncoder.setPosition(0);
    rbEncoder.setPosition(0);
    lfEncoder.setPosition(0);
    lbEncoder.setPosition(0);
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), Constants.DriveTrain.INITIAL_POS);
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
        leftPIDController.calculate(getLeftVelocity(), speeds.leftMetersPerSecond);
    final double rightOutput =
        rightPIDController.calculate(getRightVelocity(), speeds.rightMetersPerSecond);
    leftGroup.setVoltage(leftOutput + leftFeedforward);
    rightGroup.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }
  public void drive(ChassisSpeeds chassisSpeeds){
    var wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    setSpeeds(wheelSpeeds);
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(), getRightPos(), getLeftPos());
  }

  public Pose2d getFieldPosition(){
    return odometry.getPoseMeters();
  }

  public double getRotation(){
    return gyro.getAngle();
  }

  public double getRightPos(){
    return (rfEncoder.getPosition() + rbEncoder.getPosition())/2;
  }
  public double getLeftPos(){
    return (lfEncoder.getPosition() + lbEncoder.getPosition())/2;
  }

  public double getRightVelocity(){
    return (rfEncoder.getVelocity() + rbEncoder.getVelocity())/2;
  }
  public double getLeftVelocity(){
    return (lfEncoder.getVelocity() + lbEncoder.getVelocity())/2;
  }
 
}