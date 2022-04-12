// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Drive.ArcadeDrive;
import frc.robot.commands.Drive.DebugDrive;
import frc.robot.commands.Drive.DebugDrive2;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/** Represents a differential drive style drivetrain. */
public class DriveTrain extends SubsystemBase implements DriveTrainInterface{

  
  private final WPI_TalonFX rfMotor = new WPI_TalonFX(Constants.DriveTrain.RFMOTOR_ID);
  private final WPI_TalonFX rbMotor = new WPI_TalonFX(Constants.DriveTrain.RBMOTOR_ID);
  private final WPI_TalonFX lfMotor = new WPI_TalonFX(Constants.DriveTrain.LFMOTOR_ID);
  private final WPI_TalonFX lbMotor = new WPI_TalonFX(Constants.DriveTrain.LBMOTOR_ID);
  
  private final MotorControllerGroup rightGroup =
  new MotorControllerGroup(rfMotor, rbMotor);
  private final MotorControllerGroup leftGroup =
      new MotorControllerGroup(lfMotor, lbMotor);

  private AHRS gyro;

  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Constants.DriveTrain.TRACK_WIDTH);

  private DifferentialDriveOdometry odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.DriveTrain.RIGHT_kS, Constants.DriveTrain.RIGHT_kV);
  private SimpleMotorFeedforward rfeedforward = new SimpleMotorFeedforward(Constants.DriveTrain.RIGHT_kS, Constants.DriveTrain.RIGHT_kV);
  private SimpleMotorFeedforward lfeedforward = new SimpleMotorFeedforward(Constants.DriveTrain.LEFT_kS, Constants.DriveTrain.LEFT_kV);
  private SimpleMotorFeedforward lfeedbackward = new SimpleMotorFeedforward(Constants.DriveTrain.LEFT_BACKWARD_kS, Constants.DriveTrain.LEFT_BACKWARD_kV);

  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
   * gyro.
   */
  public DriveTrain() {
    try{
      gyro = new AHRS(SPI.Port.kMXP);
      
    }catch(Exception E){
      System.out.println("Error is where we thought it was");
    }

      rfMotor.setInverted(TalonFXInvertType.CounterClockwise);
      rbMotor.setInverted(TalonFXInvertType.CounterClockwise);
      lfMotor.setInverted(TalonFXInvertType.CounterClockwise);
      lbMotor.setInverted(TalonFXInvertType.CounterClockwise);

      //Should all of these be rf? This is the same in DriveTrain2
      //Who wrote this? thanks by the way
      rfMotor.setNeutralMode(NeutralMode.Brake);
      rbMotor.setNeutralMode(NeutralMode.Brake);
      lfMotor.setNeutralMode(NeutralMode.Brake);
      lbMotor.setNeutralMode(NeutralMode.Brake);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightGroup.setInverted(true);
    leftGroup.setInverted(false);
    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.

    zeroEncoders();
    odometry = new DifferentialDriveOdometry(new Rotation2d(0), Constants.DriveTrain.INITIAL_POS);
    setDefaultCommand(new ArcadeDrive(this));
  }


  public void periodic(){
    updateOdometry();
    printValues();

  }

  public void printValues(){
    SmartDashboard.putString("Measured Vels", "Right: " + round(getRightVelocity(), 2) + "; Left: " + round(getLeftVelocity(), 2));
    SmartDashboard.putString("Field Position", "xValue: " 
    + ((double)Math.round(getFieldPosition().getX() * 1000d) / 1000d)
    + " Y Value: " 
    +((double)Math.round(getFieldPosition().getY() * 1000d) / 1000d)
    + " Get Rotation: " 
    + ((double)Math.round(getFieldPosition().getRotation().getRadians() * 1000d) / 1000d));

    SmartDashboard.putNumber("Right Encoder", getRightPos());
    SmartDashboard.putNumber("Left Encoder", getLeftPos());
    // SmartDashboard.putNumber("Angle", gyro.getAngle());
    SmartDashboard.putNumber("Field Pos Angle", getFieldPosition().getRotation().getRadians());    
    SmartDashboard.putNumber("Get Angle in Rad",getRotation());

  }

  private double round(double value, double decimalPlaces){
    return ((int)(value * Math.pow(10, decimalPlaces))) / Math.pow(10,decimalPlaces);
  }

  public void  setBrakeMode(boolean isBreak){
    if(isBreak){
      rfMotor.setNeutralMode(NeutralMode.Brake);
      rbMotor.setNeutralMode(NeutralMode.Brake);
      lfMotor.setNeutralMode(NeutralMode.Brake);
      lbMotor.setNeutralMode(NeutralMode.Brake);
    }else{
      rfMotor.setNeutralMode(NeutralMode.Coast);
      rbMotor.setNeutralMode(NeutralMode.Coast);
      lfMotor.setNeutralMode(NeutralMode.Coast);
      lbMotor.setNeutralMode(NeutralMode.Coast);
    }
  }


  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double rightFeedforward = rfeedforward.calculate(speeds.rightMetersPerSecond);
    double leftFeedforward = 0;

    // if(speeds.leftMetersPerSecond >= 0){
      if(true){
      leftFeedforward = lfeedforward.calculate(speeds.leftMetersPerSecond);
    }else{
      leftFeedforward = lfeedbackward.calculate(speeds.leftMetersPerSecond);
    }

    leftGroup.setVoltage(MathUtil.clamp(leftFeedforward,-12,12));
    rightGroup.setVoltage(MathUtil.clamp(rightFeedforward,-12,12));
  }

  public void setSpeeds(double leftSpeed, double rightSpeed){
    leftGroup.setVoltage(leftSpeed);
    rightGroup.setVoltage(rightSpeed);
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

  public void debugDrive(double kS, double backwardkS, double kV, double backwardkV, double xSpeed, double rot){
    var speeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    final double rightFeedforward = rfeedforward.calculate(speeds.rightMetersPerSecond);
    var localLFF = new SimpleMotorFeedforward(kS, kV);
    var localBLFF = new SimpleMotorFeedforward(backwardkS, backwardkV);

    double leftFeedforward = 0;

    if(speeds.leftMetersPerSecond >= 0){
      leftFeedforward = localLFF.calculate(speeds.leftMetersPerSecond);
    }else{
      leftFeedforward = localBLFF.calculate(speeds.leftMetersPerSecond);
    }
    leftGroup.set(MathUtil.clamp(leftFeedforward,-12,12)/12);
    rightGroup.set(MathUtil.clamp(rightFeedforward,-12,12)/12);
  }


  public void debugDrive(double xSpeed, double rot){
    var speeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    // double rightSpeed = feedforward.calculate(speeds.rightMetersPerSecond);
    // double leftSpeed = feedforward.calculate(speeds.leftMetersPerSecond) * Constants.DriveTrain.LEFT_SPEED_CONSTANT;
    double rightSpeed = speeds.rightMetersPerSecond;
    double leftSpeed = speeds.leftMetersPerSecond * Constants.DriveTrain.LEFT_SPEED_CONSTANT;
    rightSpeed = MathUtil.clamp(rightSpeed,-12,12)/12;
    leftSpeed = MathUtil.clamp(leftSpeed,-12,12)/12;

    rightGroup.set(rightSpeed);
    leftGroup.set(leftSpeed);
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(), getLeftPos(), getRightPos());
  }

  public Pose2d getFieldPosition(){
    return odometry.getPoseMeters();
  }

  public double getRotation(){
    return -gyro.getAngle();
  }

  public double getRightPos(){
    return (rfMotor.getSelectedSensorPosition() +rbMotor.getSelectedSensorPosition())/2 * Constants.DriveTrain.WHEEL_CIRCUMFERENCE/ 2048 /12.75;
  }
  public double getLeftPos(){
    return -(lfMotor.getSelectedSensorPosition() + lbMotor.getSelectedSensorPosition())/2 * Constants.DriveTrain.WHEEL_CIRCUMFERENCE / 2048 / 12.75;
  }

  public double getRightRPM(){
    return -(rfMotor.getSelectedSensorVelocity() + rbMotor.getSelectedSensorVelocity())/2 * 10;
  }

  public double getLeftRPM(){
    return -(lfMotor.getSelectedSensorVelocity() + lbMotor.getSelectedSensorVelocity())/2 * 10;
  }
  public double getRightVelocity(){
    return (rfMotor.getSelectedSensorVelocity() + rbMotor.getSelectedSensorVelocity())/2 * Constants.DriveTrain.WHEEL_CIRCUMFERENCE / 2048 / 12.75 * 10;
  }
  public double getLeftVelocity(){
    return -(lfMotor.getSelectedSensorVelocity() + lbMotor.getSelectedSensorVelocity())/2 * Constants.DriveTrain.WHEEL_CIRCUMFERENCE / 2048 / 12.75  *10;
  }

  public void setRF(double speed){
    rfMotor.set(speed);
  }
  public void setRB(double speed){
    rbMotor.set(speed);
  }
  public void setLF(double speed){
    lfMotor.set(speed);
  }
  public void setLB(double speed){
    lbMotor.set(speed);
  }

  public void zeroEncoders(){
    rfMotor.setSelectedSensorPosition(0);
    rbMotor.setSelectedSensorPosition(0);
    lfMotor.setSelectedSensorPosition(0);
    lbMotor.setSelectedSensorPosition(0);
    gyro.reset();
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(),new Pose2d(0,0, new Rotation2d(0)));
  }

  public void setFieldPos(Pose2d currentPos){
    zeroEncoders();
    gyro.reset();
    gyro.setAngleAdjustment(-currentPos.getRotation().getDegrees());
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), currentPos);
  }
}