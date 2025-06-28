// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

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


/** Represents a differential drive style drivetrain. */
public class DriveTrain extends SubsystemBase implements DriveTrainInterface{

  
  private final TalonFX rfMotor = new TalonFX(Constants.DriveTrain.RFMOTOR_ID);
  private final TalonFX rbMotor = new TalonFX(Constants.DriveTrain.RBMOTOR_ID);
  private final TalonFX lfMotor = new TalonFX(Constants.DriveTrain.LFMOTOR_ID);
  private final TalonFX lbMotor = new TalonFX(Constants.DriveTrain.LBMOTOR_ID);
  
  private AHRS gyro;

  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Constants.DriveTrain.TRACK_WIDTH);

  private DifferentialDriveOdometry odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  @SuppressWarnings("unused")
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.DriveTrain.RIGHT_kS, Constants.DriveTrain.RIGHT_kV);
  private SimpleMotorFeedforward rfeedforward = new SimpleMotorFeedforward(Constants.DriveTrain.RIGHT_kS, Constants.DriveTrain.RIGHT_kV);
  private SimpleMotorFeedforward lfeedforward = new SimpleMotorFeedforward(Constants.DriveTrain.LEFT_kS, Constants.DriveTrain.LEFT_kV);
  private SimpleMotorFeedforward lfeedbackward = new SimpleMotorFeedforward(Constants.DriveTrain.LEFT_BACKWARD_kS, Constants.DriveTrain.LEFT_BACKWARD_kV);

  TalonFXConfiguration rfConfig;
  TalonFXConfiguration rbConfig;
  TalonFXConfiguration lfConfig;
  TalonFXConfiguration lbConfig;

  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
   * gyro.
   */
  public DriveTrain() {
    try{
      gyro = new AHRS(NavXComType.kMXP_SPI);
      
    }catch(Exception E){
      System.out.println("Error is where we thought it was");
    }


    rfConfig = new TalonFXConfiguration();
    rbConfig = new TalonFXConfiguration();
    lfConfig = new TalonFXConfiguration();
    lbConfig = new TalonFXConfiguration();

    rfConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    lfConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    lbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    lfConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    lbConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rfConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rbConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    lfMotor.getConfigurator().apply(lfConfig);
    lbMotor.getConfigurator().apply(lbConfig);
    rfMotor.getConfigurator().apply(rfConfig);
    rbMotor.getConfigurator().apply(rbConfig);

    rbMotor.setControl(new Follower(rfMotor.getDeviceID(), false));
    lbMotor.setControl(new Follower(lfMotor.getDeviceID(), false));

    setRampRate(false);


    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.

    zeroEncoders();
    odometry = new DifferentialDriveOdometry(new Rotation2d(0), getLeftPos(), getRightPos(), Constants.DriveTrain.INITIAL_POS);
    setDefaultCommand(new ArcadeDrive(this));
    SmartDashboard.putNumber("rampRate", 0);
  }


  public void periodic(){
    updateOdometry();
    printValues();
  }

  public void setRampRate(boolean isTrue){
    double rr = 0;
    if(isTrue){
      rr = 0.7;
    }else{
      rr = 0;
    }
    setRampRate(rr);
  }

  public void setRampRate(double rr){
    // lfMotor.getConfigurator().apply(lfConfig.OpenLoopRamps.withVoltageOpenLoopRampPeriod(rr));
    // lbMotor.getConfigurator().apply(lbConfig.OpenLoopRamps.withVoltageOpenLoopRampPeriod(rr));
    // rfMotor.getConfigurator().apply(rfConfig.OpenLoopRamps.withVoltageOpenLoopRampPeriod(rr));
    // rbMotor.getConfigurator().apply(rbConfig.OpenLoopRamps.withVoltageOpenLoopRampPeriod(rr));
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
    lfMotor.getConfigurator().apply(lfConfig.MotorOutput.withNeutralMode(isBreak ? NeutralModeValue.Brake : NeutralModeValue.Coast));
    lbMotor.getConfigurator().apply(lbConfig.MotorOutput.withNeutralMode(isBreak ? NeutralModeValue.Brake : NeutralModeValue.Coast));
    rfMotor.getConfigurator().apply(rfConfig.MotorOutput.withNeutralMode(isBreak ? NeutralModeValue.Brake : NeutralModeValue.Coast));
    rbMotor.getConfigurator().apply(rbConfig.MotorOutput.withNeutralMode(isBreak ? NeutralModeValue.Brake : NeutralModeValue.Coast));
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

    System.out.println(leftFeedforward+ ", " + rightFeedforward);

    lfMotor.setVoltage(MathUtil.clamp(leftFeedforward,-12,12));
    rfMotor.setVoltage(MathUtil.clamp(rightFeedforward,-12,12));
  }

  public void setSpeeds(double leftSpeed, double rightSpeed){
    System.out.println("noop setspeeds called");
    // lfMotor.setVoltage(leftSpeed);
    // rfMotor.setVoltage(rightSpeed);
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
    // lfMotor.set(MathUtil.clamp(leftFeedforward,-12,12)/12);
    // rfMotor.set(MathUtil.clamp(rightFeedforward,-12,12)/12);
    System.out.println("noop debugdrive1 called");
  }


  public void debugDrive(double xSpeed, double rot){
    var speeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    // double rightSpeed = feedforward.calculate(speeds.rightMetersPerSecond);
    // double leftSpeed = feedforward.calculate(speeds.leftMetersPerSecond) * Constants.DriveTrain.LEFT_SPEED_CONSTANT;
    double rightSpeed = speeds.rightMetersPerSecond;
    double leftSpeed = speeds.leftMetersPerSecond * Constants.DriveTrain.LEFT_SPEED_CONSTANT;
    rightSpeed = MathUtil.clamp(rightSpeed,-12,12)/12;
    leftSpeed = MathUtil.clamp(leftSpeed,-12,12)/12;

    // rfMotor.set(rightSpeed);
    // lfMotor.set(leftSpeed);
    System.out.println("nooop debugdrive2 called");
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
    return (rfMotor.getPosition().getValueAsDouble() + rbMotor.getPosition().getValueAsDouble())/2 * Constants.DriveTrain.WHEEL_CIRCUMFERENCE /12.75;
  }
  public double getLeftPos(){
    return -(lbMotor.getPosition().getValueAsDouble() + lfMotor.getPosition().getValueAsDouble())/2 * Constants.DriveTrain.WHEEL_CIRCUMFERENCE/ 12.75;
  }

  public double getRightRPM(){
    return (rfMotor.getVelocity().getValueAsDouble() + rbMotor.getVelocity().getValueAsDouble())/2 * 60;
  }

  public double getLeftRPM(){
    return -(lfMotor.getVelocity().getValueAsDouble() + lbMotor.getVelocity().getValueAsDouble())/2 * 60;
  }

  public double getRightVelocity(){
    return (rfMotor.getVelocity().getValueAsDouble() + rbMotor.getVelocity().getValueAsDouble())/2 * Constants.DriveTrain.WHEEL_CIRCUMFERENCE / 12.75;
  }
  public double getLeftVelocity(){
    return -(lfMotor.getVelocity().getValueAsDouble() + lbMotor.getVelocity().getValueAsDouble())/2 * Constants.DriveTrain.WHEEL_CIRCUMFERENCE / 12.75;
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
    rfMotor.setPosition(0);
    rbMotor.setPosition(0);
    lfMotor.setPosition(0);
    lbMotor.setPosition(0);
    gyro.reset();
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0, new Pose2d(0,0, new Rotation2d(0)));
  }

  public void setFieldPos(Pose2d currentPos){
    zeroEncoders();
    gyro.reset();
    gyro.setAngleAdjustment(-currentPos.getRotation().getDegrees());
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0, currentPos);
  }
}