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
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DebugDrive;
import frc.robot.commands.DebugDrive2;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/** Represents a differential drive style drivetrain. */
public class DriveTrain extends SubsystemBase{

  // private final CANSparkMax rfMotor = new CANSparkMax(Constants.DriveTrain.RFMOTOR_ID, MotorType.kBrushless);
  // private final CANSparkMax rbMotor = new CANSparkMax(Constants.DriveTrain.RBMOTOR_ID, MotorType.kBrushless);
  // private final CANSparkMax lfMotor = new CANSparkMax(Constants.DriveTrain.LFMOTOR_ID, MotorType.kBrushless);
  // private final CANSparkMax lbMotor = new CANSparkMax(Constants.DriveTrain.LBMOTOR_ID, MotorType.kBrushless);
  
  private final WPI_TalonFX rfMotor = new WPI_TalonFX(Constants.DriveTrain.RFMOTOR_ID);
  private final WPI_TalonFX rbMotor = new WPI_TalonFX(Constants.DriveTrain.RBMOTOR_ID);
  private final WPI_TalonFX lfMotor = new WPI_TalonFX(Constants.DriveTrain.LFMOTOR_ID);
  private final WPI_TalonFX lbMotor = new WPI_TalonFX(Constants.DriveTrain.LBMOTOR_ID);


  // private final RelativeEncoder rfEncoder = rfMotor.getEncoder();
  // private final RelativeEncoder rbEncoder = rbMotor.getEncoder();
  // private final RelativeEncoder lfEncoder = lfMotor.getEncoder();
  // private final RelativeEncoder lbEncoder = lbMotor.getEncoder();
  
  private final MotorControllerGroup rightGroup =
  new MotorControllerGroup(rfMotor, rbMotor);
  private final MotorControllerGroup leftGroup =
      new MotorControllerGroup(lfMotor, lbMotor);

  private AHRS gyro;

  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Constants.DriveTrain.TRACK_WIDTH);

  private DifferentialDriveOdometry odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward rfeedforward = new SimpleMotorFeedforward(Constants.DriveTrain.RIGHT_kS, Constants.DriveTrain.RIGHT_kV);
  private final SimpleMotorFeedforward lfeedforward = new SimpleMotorFeedforward(Constants.DriveTrain.LEFT_kS, Constants.DriveTrain.LEFT_kV);
  private final SimpleMotorFeedforward lfeedbackward = new SimpleMotorFeedforward(Constants.DriveTrain.LEFT_BACKWARD_kS, Constants.DriveTrain.LEFT_BACKWARD_kV);

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
    
    TalonFXConfiguration configs = new TalonFXConfiguration();
			/* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
			configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
			/* config all the settings */
			rfMotor.configAllSettings(configs);
      rbMotor.configAllSettings(configs);
      lfMotor.configAllSettings(configs);
      lbMotor.configAllSettings(configs);
      
      rfMotor.setInverted(TalonFXInvertType.CounterClockwise);
      rbMotor.setInverted(TalonFXInvertType.CounterClockwise);
      lfMotor.setInverted(TalonFXInvertType.CounterClockwise);
      lbMotor.setInverted(TalonFXInvertType.CounterClockwise);

      //Should all of these be rf? This is the same in DriveTrain2
      rfMotor.setNeutralMode(NeutralMode.Brake);
      rfMotor.setNeutralMode(NeutralMode.Brake);
      rfMotor.setNeutralMode(NeutralMode.Brake);
      rfMotor.setNeutralMode(NeutralMode.Brake);

      rfMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
      rbMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
      lfMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
      lbMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);


    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightGroup.setInverted(false);
    leftGroup.setInverted(false);
    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.

    zeroEncoders();
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), Constants.DriveTrain.INITIAL_POS);
    setDefaultCommand(new ArcadeDrive(this));
  }


  public void periodic(){
    // SmartDashboard.putString("Field Position", "xValue: " 
    // + ((double)Math.round(getFieldPosition().getX() * 1000d) / 1000d)
    // + " Y Value: " 
    // +((double)Math.round(getFieldPosition().getY() * 1000d) / 1000d)
    // + " Get Rotation: " 
    // + ((double)Math.round(getFieldPosition().getRotation().getRadians() * 1000d) / 1000d));
    updateOdometry();
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
    leftGroup.set(MathUtil.clamp(leftFeedforward,-12,12)/12);
    rightGroup.set(MathUtil.clamp(rightFeedforward,-12,12)/12);
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
    return (rfMotor.getSelectedSensorPosition() +rbMotor.getSelectedSensorPosition())/2  * Constants.DriveTrain.WHEEL_CIRCUMFERENCE / 2048;
  }
  public double getLeftPos(){
    return (lfMotor.getSelectedSensorPosition() + lbMotor.getSelectedSensorPosition())/2 * Constants.DriveTrain.WHEEL_CIRCUMFERENCE / 2048;
  }

  public double getRightVelocity(){
    return (rfMotor.getSelectedSensorVelocity() + rfMotor.getSelectedSensorVelocity())/2 * Constants.DriveTrain.WHEEL_CIRCUMFERENCE / 2048;
  }
  public double getLeftVelocity(){
    return (rfMotor.getSelectedSensorVelocity() + rfMotor.getSelectedSensorVelocity())/2 * Constants.DriveTrain.WHEEL_CIRCUMFERENCE / 2048;
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
    gyro.setAngleAdjustment(gyro.getAngle());
    odometry = new DifferentialDriveOdometry(new Rotation2d(0), Constants.DriveTrain.INITIAL_POS);
  }
}