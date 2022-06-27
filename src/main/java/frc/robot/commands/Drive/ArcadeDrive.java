// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleUnaryOperator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainInterface;

  public class ArcadeDrive extends CommandBase {
  private DriveTrainInterface driveTrain;
  private XboxController controller = new XboxController(Constants.Controller.PORT);
  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Constants.DriveTrain.TRACK_WIDTH);
  private double prevXSpeed = 0;
  private double prevRotSpeed = 0;
  private double prevLVAxis = 0;
  private double maxAcc = 0.1;
  SlewRateLimiter filter = new SlewRateLimiter(4.5);
  SlewRateLimiter rotFilter = new SlewRateLimiter(3.5);
  private DoubleUnaryOperator accFormula = (vel)->{
    double minVal = 0.5;
    double maxVal = 0.7;
    double steepness = 3;
    double meanX = 2.5;
    double result = (maxVal-minVal)/(1+Math.pow(Math.E, steepness * (vel-meanX)))+minVal;
    return result;
  };

  private DoubleUnaryOperator experimentalAccFormula = (vel)->{
    double c1 = -0.0008347,
           c2 = 5.02779,
           c3 = 0.503211,
           c4 = 0.0396789,
           c5 = -1.40229;

    double result = c1*Math.pow(vel,c2) + c3 + c4 * Math.pow(Math.E, vel * c5);
    result = Math.max(Math.abs(result), 0.3);
    return result;
  };

  /** Creates a new ArcadeDriveC. */
  public ArcadeDrive(DriveTrainInterface driveTrain) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    SmartDashboard.putNumber("Min Val", SmartDashboard.getNumber("Min Val", 0.5));
    SmartDashboard.putNumber("Max Val", SmartDashboard.getNumber("Max Val", 0.7));
    SmartDashboard.putNumber("steepness", SmartDashboard.getNumber("steepness", 3));
    SmartDashboard.putNumber("meanXSpeed", SmartDashboard.getNumber("meanXSpeed", 1.5));
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    prevXSpeed = 0;
    prevRotSpeed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Left Speed", driveTrain.getLeftVelocity());
    SmartDashboard.putNumber("Right Speed", driveTrain.getRightVelocity());
    // if((Math.abs(driveTrain.getLeftVelocity()) + Math.abs(driveTrain.getRightVelocity())) / 2 > 2){
    //   driveTrain.setRampRate(0.5);
    // }else{
    //   driveTrain.setRampRate(0.7);
    // }
    
    // double vel = (Math.abs(driveTrain.getLeftVelocity()) + Math.abs(driveTrain.getRightVelocity())) / 2;
    // SmartDashboard.putNumber("Velocity for Ramprate", vel);
    // double minVal = SmartDashboard.getNumber("Min Val", 0.5);
    // double maxVal =  SmartDashboard.getNumber("Max Val", 0.7);
    // double steepness =  SmartDashboard.getNumber("steepness", 3);
    // double meanX =  SmartDashboard.getNumber("meanXSpeed", 1.5);
    // double result = (maxVal-minVal)/(1+Math.pow(Math.E, steepness * (vel-meanX)))+minVal;
    // driveTrain.setRampRate(experimentalAccFormula.applyAsDouble(vel));

    // ChassisSpeeds prevSpeeds = kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(driveTrain.getLeftVelocity(),driveTrain.getRightVelocity()));
    double lvAxis = Constants.DriveTrain.FORWARD_DIRECTION  * controller.getLeftY();
    double rhAxis = -controller.getRightX();

    lvAxis = MathUtil.applyDeadband(lvAxis, 0.08);
    rhAxis = MathUtil.applyDeadband(rhAxis, 0.08);

    lvAxis*= Math.abs(lvAxis);
    rhAxis*= Math.abs(rhAxis);

    double xSpeed = lvAxis * Constants.DriveTrain.MAX_VELOCITY;
    double rotSpeed = rhAxis * Constants.DriveTrain.MAX_ROT_VELOCITY;

    // double changeX = Math.min(Math.abs(xSpeed - prevXSpeed), acc.applyAsDouble(Math.abs(prevXSpeed), 0.0));
    // double direction = (xSpeed - prevXSpeed) / Math.abs(xSpeed - prevXSpeed);

    // xSpeed = prevXSpeed + changeX * direction;



    SmartDashboard.putNumber("X Speed HERE", filter.calculate(xSpeed));
    SmartDashboard.putNumber("rot Speed man", rotFilter.calculate(rotSpeed));


    double rightTrigger = controller.getRightTriggerAxis();
    if(rightTrigger > 0.5){
      Constants.DriveTrain.MAX_VELOCITY = 3;
    }else{
      Constants.DriveTrain.MAX_VELOCITY = 2;
    }
    // if(xSpeed == 0){
    //   SmartDashboard.putBoolean("Working", true);
    //   SmartDashboard.putNumber("prevXSpeed", prevXSpeed);
    //   if(Math.abs(xSpeed - prevXSpeed) > 0.2){
    //     double changeSpeed = 0.2;
    //     double changeVel = ((xSpeed - prevXSpeed)/Math.abs(xSpeed - prevXSpeed)) *  changeSpeed;
    //     SmartDashboard.putNumber("changeVel", changeVel);
    //     xSpeed = prevXSpeed + changeVel;
    //   }else{
    //     xSpeed = xSpeed;
    //     SmartDashboard.putNumber("changeVel", 0);
    //   }
    // }else{
    //   SmartDashboard.putBoolean("Working", false);
    // }

    // SmartDashboard.putNumber("xSpeed here", xSpeed);

    // driveTrain.drive(filter.calculate(xSpeed), rotSpeed);

    driveTrain.drive(xSpeed, rotSpeed);


    // SmartDashboard.putString("Speeds", "prev: " + round(prevRotSpeed, 2) + "; rot: " + round(rotSpeed, 2));
    // SmartDashboard.putString("x Speeds", "prev: " + round(prevXSpeed, 2) + "; x: " + round(xSpeed, 2));
    
    prevLVAxis = lvAxis;
    prevXSpeed = xSpeed;
    prevRotSpeed = rotSpeed;
  }

  private double round(double value, double decimalPlaces){
    return ((int)(value * Math.pow(10, decimalPlaces))) / Math.pow(10,decimalPlaces);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
