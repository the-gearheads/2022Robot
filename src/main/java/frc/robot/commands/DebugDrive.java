// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DebugDrive extends CommandBase {
  private DriveTrain driveTrain;

  /** Creates a new DebugDrive. */
  public DebugDrive(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("backward kV", Constants.DriveTrain.LEFT_BACKWARD_kV);
    SmartDashboard.putNumber("kV", Constants.DriveTrain.LEFT_kV);
    SmartDashboard.putNumber("backward kS", Constants.DriveTrain.LEFT_BACKWARD_kS);
    SmartDashboard.putNumber("kS", Constants.DriveTrain.LEFT_kS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double bkV = SmartDashboard.getNumber("backward kV", 0);
    double kV = SmartDashboard.getNumber("kV", 0);
    double bkS = SmartDashboard.getNumber("backward kS", 0);
    double kS = SmartDashboard.getNumber("kS", 0);

    double lvAxis = -DriverStation.getStickAxis(Constants.Controller.PORT, Constants.Controller.LV_AXIS);
    double rhAxis = -DriverStation.getStickAxis(Constants.Controller.PORT, Constants.Controller.RH_AXIS);

    driveTrain.debugDrive(kS, bkS, kV, bkV, lvAxis, rhAxis);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.debugDrive(0,0, 0, 0, 0, 0);

  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
