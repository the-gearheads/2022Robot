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
import frc.robot.commands.Drive.FeedForwardCharacterization1;

/** Represents a differential drive style drivetrain. */
public class DriveTrain2 extends SubsystemBase implements DriveTrainInterface{

  private final CANSparkMax rfMotor = new CANSparkMax(Constants.DriveTrain.TEST_RFMOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax rbMotor = new CANSparkMax(Constants.DriveTrain.TEST_RBMOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax lfMotor = new CANSparkMax(Constants.DriveTrain.TEST_LFMOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax lbMotor = new CANSparkMax(Constants.DriveTrain.TEST_LBMOTOR_ID, MotorType.kBrushless);
  


  private final RelativeEncoder rfEncoder = rfMotor.getEncoder();
  private final RelativeEncoder rbEncoder = rbMotor.getEncoder();
  private final RelativeEncoder lfEncoder = lfMotor.getEncoder();
  private final RelativeEncoder lbEncoder = lbMotor.getEncoder();
  
  private final MotorControllerGroup rightGroup =
  new MotorControllerGroup(rfMotor, rbMotor);
  private final MotorControllerGroup leftGroup =
      new MotorControllerGroup(lfMotor, lbMotor);

  private AHRS gyro;

  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Constants.DriveTrain.TRACK_WIDTH);

  private DifferentialDriveOdometry odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward rfeedforward = new SimpleMotorFeedforward(Constants.DriveTrain.TEST_RIGHT_kS, Constants.DriveTrain.TEST_RIGHT_kV);
  private final SimpleMotorFeedforward lfeedforward = new SimpleMotorFeedforward(Constants.DriveTrain.TEST_LEFT_kS, Constants.DriveTrain.TEST_LEFT_kV);
  private final SimpleMotorFeedforward lfeedbackward = new SimpleMotorFeedforward(Constants.DriveTrain.TEST_LEFT_BACKWARD_kS, Constants.DriveTrain.TEST_LEFT_BACKWARD_kV);

  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
   * gyro.
   */
  public DriveTrain2() {
    try{
      gyro = new AHRS(SPI.Port.kMXP);
      
    }catch(Exception E){
      System.out.println("Error is where we thought it was");
    }
      
      rfMotor.setInverted(false);
      rbMotor.setInverted(false);
      lfMotor.setInverted(false);
      lbMotor.setInverted(false);

      setBrakeMode(true);

      lbEncoder.setVelocityConversionFactor(1);
      lfEncoder.setVelocityConversionFactor(1);      
      rbEncoder.setVelocityConversionFactor(1);
      rfEncoder.setVelocityConversionFactor(1);

      lbEncoder.setPositionConversionFactor(1);
      lfEncoder.setPositionConversionFactor(1);      
      rbEncoder.setPositionConversionFactor(1);
      rfEncoder.setPositionConversionFactor(1);


    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightGroup.setInverted(true);
    leftGroup.setInverted(false);
    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.

    zeroEncoders();
    SmartDashboard.putString("Initial Angle", gyro.getRotation2d() + "");
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), new Pose2d(0,0,new Rotation2d(0)));
    setDefaultCommand(new ArcadeDrive(this));

    SmartDashboard.putNumber("Velocity", 0.1);
    SmartDashboard.putNumber("Acc", 0.1);
  }


  public void periodic(){
    updateOdometry();
    printValues();
    // SmartDashboard.putNumber("Gyro", gyro.getRotation2d().getRadians());

  }

  public void printValues(){
    
    SmartDashboard.putString("Field Position", "xValue: " 
    + ((double)Math.round(getFieldPosition().getX() * 1000d) / 1000d)
    + " Y Value: " 
    +((double)Math.round(getFieldPosition().getY() * 1000d) / 1000d)
    + " Get Rotation: " 
    + ((double)Math.round(getFieldPosition().getRotation().getRadians() * 1000d) / 1000d));

    SmartDashboard.putNumber("Right Speed", getRightVelocity());

    // SmartDashboard.putNumber("Right Encoder", getRightPos());
    // SmartDashboard.putNumber("Left Encoder", getLeftPos());
    // SmartDashboard.putNumber("Angle", gyro.getAngle());
    // SmartDashboard.putNumber("Field Pos Angle", getFieldPosition().getRotation().getRadians());
  }


  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double rightFeedforward = rfeedforward.calculate(speeds.rightMetersPerSecond);
    

    double leftFeedforward = 0;
    if(speeds.leftMetersPerSecond >= 0){
      leftFeedforward = lfeedforward.calculate(speeds.leftMetersPerSecond);
    }else{
      leftFeedforward = lfeedbackward.calculate(speeds.leftMetersPerSecond);
    }

    // SmartDashboard.putString("Feedforward values", "Left: " + speeds.leftMetersPerSecond  +  "; Right: " + speeds.rightMetersPerSecond);
    leftGroup.setVoltage(MathUtil.clamp(leftFeedforward,-12,12));
    rightGroup.setVoltage(MathUtil.clamp(rightFeedforward,-12,12));
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
    // SmartDashboard.putString("Wheel Speeds", "Left: " + wheelSpeeds.leftMetersPerSecond  +  "; Right: " + wheelSpeeds.rightMetersPerSecond);
    setSpeeds(wheelSpeeds);
  }
  public void drive(ChassisSpeeds chassisSpeeds){
    var wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    setSpeeds(wheelSpeeds);
  }

  public void debugDrive(double kS, double backwardkS, double kV, double backwardkV, double xSpeed, double rot){
    var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
    SimpleMotorFeedforward localLFF = new SimpleMotorFeedforward(kS, kV);
    SimpleMotorFeedforward localBLFF = new SimpleMotorFeedforward(backwardkS, backwardkV);
    final double rightFeedforward = rfeedforward.calculate(wheelSpeeds.rightMetersPerSecond);
    
    double leftFeedforward = 0;
    if(wheelSpeeds.leftMetersPerSecond >= 0){
      leftFeedforward = localLFF.calculate(wheelSpeeds.leftMetersPerSecond);
    }else{
      leftFeedforward = localBLFF.calculate(wheelSpeeds.leftMetersPerSecond);
    }
    leftGroup.set(MathUtil.clamp(leftFeedforward,-12,12)/12);
    rightGroup.set(MathUtil.clamp(rightFeedforward,-12,12)/12);
    
  }

  public void debugDrive(double xSpeed, double rot){
    //FILL PLZ
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
    return -(rfEncoder.getPosition() +rbEncoder.getPosition())/2  * Constants.DriveTrain.TEST_WHEEL_CIRCUMFERENCE/ 7.3;
  }
  public double getLeftPos(){
    return (lfEncoder.getPosition() + lbEncoder.getPosition())/2 * Constants.DriveTrain.TEST_WHEEL_CIRCUMFERENCE / 7.3;
  }

  public double getRightVelocity(){
    return -(rfEncoder.getVelocity() + rfEncoder.getVelocity())/2 * Constants.DriveTrain.TEST_WHEEL_CIRCUMFERENCE / 7.3 / 60;
  }
  public double getLeftVelocity(){
    return -(rfEncoder.getVelocity() + rfEncoder.getVelocity())/2 * Constants.DriveTrain.TEST_WHEEL_CIRCUMFERENCE / 7.3 / 60;
  }

  public void setSpeeds(double leftSpeed, double rightSpeed){
    leftGroup.setVoltage(leftSpeed);
    rightGroup.setVoltage(rightSpeed);
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
    rfEncoder.setPosition(0);
    rbEncoder.setPosition(0);
    lfEncoder.setPosition(0);
    lbEncoder.setPosition(0);
    gyro.reset();
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(),new Pose2d(0,0, new Rotation2d(0)));
  }

  public void setFieldPos(Pose2d currentPos){
    zeroEncoders();
    gyro.reset();
    gyro.setAngleAdjustment(-currentPos.getRotation().getDegrees());
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), currentPos);
  }
  public void setBrakeMode(boolean isBreak){
    if(isBreak){
      rfMotor.setIdleMode(IdleMode.kBrake);
      rbMotor.setIdleMode(IdleMode.kBrake);
      lfMotor.setIdleMode(IdleMode.kBrake);
      lbMotor.setIdleMode(IdleMode.kBrake);
    }else{
      rfMotor.setIdleMode(IdleMode.kCoast);
      rbMotor.setIdleMode(IdleMode.kCoast);
      lfMotor.setIdleMode(IdleMode.kCoast);
      lbMotor.setIdleMode(IdleMode.kCoast);
    }
  }
}