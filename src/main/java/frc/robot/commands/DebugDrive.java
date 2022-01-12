// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSS;

public class DebugDrive extends CommandBase {
  private DriveTrainSS driveTrainSS;

  /** Creates a new DebugDrive. */
  public DebugDrive(DriveTrainSS driveTrainSS) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrainSS = driveTrainSS;
    addRequirements(driveTrainSS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("kS", 0);
    SmartDashboard.putNumber("kV", 0);
    SmartDashboard.putNumber("kP", 0);
    SmartDashboard.putNumber("kI", 0);
    SmartDashboard.putNumber("kD", 0);

    SmartDashboard.putNumber("Right Velocity", 0);
    SmartDashboard.putNumber("Left Velocity", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Encoder Right Velocity", driveTrainSS.getRightVelocity());
    SmartDashboard.putNumber("Encoder Left Velocity", driveTrainSS.getLeftVelocity());
    SmartDashboard.putNumber("Gyroscope Rotation", driveTrainSS.getRotation());
    SmartDashboard.putNumber("Odometry Rotation", driveTrainSS.getFieldPosition().getRotation().getRadians());

    double kS = SmartDashboard.getNumber("kS", 0);
    double kV = SmartDashboard.getNumber("kV", 0);
    double kP = SmartDashboard.getNumber("kP", 0);
    double kI = SmartDashboard.getNumber("kI", 0);
    double kD = SmartDashboard.getNumber("kD", 0);

    double rightSpeed = SmartDashboard.getNumber("Right Velocity", 0);
    double leftSpeed = SmartDashboard.getNumber("Left Velocity", 0);

    driveTrainSS.debugDrive(kS, kV, kP, kI, kD, rightSpeed, leftSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSS.debugDrive(0, 0, 0, 0, 0, 0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
