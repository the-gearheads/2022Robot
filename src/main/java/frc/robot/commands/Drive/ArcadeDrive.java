// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
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

  /** Creates a new ArcadeDriveC. */
  public ArcadeDrive(DriveTrainInterface driveTrain) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
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
    // ChassisSpeeds prevSpeeds = kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(driveTrain.getLeftVelocity(),driveTrain.getRightVelocity()));
    double lvAxis = Constants.DriveTrain.FORWARD_DIRECTION  * controller.getLeftY();
    double rhAxis = -controller.getRightX();

    lvAxis = MathUtil.applyDeadband(lvAxis, 0.2);
    rhAxis = MathUtil.applyDeadband(rhAxis, 0.2);

    double xSpeed = lvAxis * Constants.DriveTrain.MAX_VELOCITY;
    double rotSpeed = rhAxis * Constants.DriveTrain.MAX_ROT_VELOCITY;

    double rightTrigger = controller.getRightTriggerAxis();
    // if(true){
    //   SmartDashboard.putBoolean("Working", true);
    //   SmartDashboard.putNumber("prevXSpeed", prevXSpeed);
    //   if(Math.abs(xSpeed - prevXSpeed) > 0.1){
    //     double changeSpeed = 0.1;
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
    if(lvAxis == 0 && prevXSpeed < 0 && Math.abs(prevXSpeed) > maxAcc){
      xSpeed = prevXSpeed + maxAcc;
    }

    // SmartDashboard.putNumber("xSpeed here", xSpeed);

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
