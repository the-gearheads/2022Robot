// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DebugDrive2 extends Command {
  private DriveTrain driveTrain;

  /** Creates a new DebugDrive2. */
  public DebugDrive2(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("rf",0);
    SmartDashboard.putNumber("rb",0);
    SmartDashboard.putNumber("lf",0);
    SmartDashboard.putNumber("lb",0);
    SmartDashboard.putBoolean("Zero?", true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean zero = SmartDashboard.getBoolean("Zero?", false);
    if(zero){
      SmartDashboard.putNumber("rf",0);
      SmartDashboard.putNumber("rb",0);
      SmartDashboard.putNumber("lf",0);
      SmartDashboard.putNumber("lb",0);
    }
    double rfSpeed = SmartDashboard.getNumber("rf", 0);
    double rbSpeed = SmartDashboard.getNumber("rb", 0);
    double lfSpeed = SmartDashboard.getNumber("lf", 0);
    double lbSpeed = SmartDashboard.getNumber("lb", 0);

    driveTrain.setRF(rfSpeed);
    driveTrain.setRB(rbSpeed);
    driveTrain.setLF(lfSpeed);
    driveTrain.setLB(lbSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setRF(0);
    driveTrain.setRB(0);
    driveTrain.setLF(0);
    driveTrain.setLB(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
