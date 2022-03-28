// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AutonChooser extends SubsystemBase {
  /** Creates a new AutonChooser. */
  public Pose2d prevPos = new Pose2d();
  public SendableChooser<Pose2d> initialPosChooser = new SendableChooser<>();
  public SendableChooser<String> autonChooser = new SendableChooser<>();
  public AutonChooser() {
    initialPosChooser.setDefaultOption("Right", Constants.Field.RIGHT_POS);
    initialPosChooser.addOption("Mid", Constants.Field.MID_POS);
    initialPosChooser.addOption("Left", Constants.Field.LEFT_POS);
    initialPosChooser.addOption("Generic", Constants.Field.ZERO);

    SmartDashboard.putData("Initial Position", initialPosChooser);
  }

  @Override
  public void periodic() {
    if(Constants.Field.RIGHT_POS.equals(initialPosChooser.getSelected()) && !prevPos.equals(Constants.Field.RIGHT_POS)){
      autonChooser  = new SendableChooser<>();
      autonChooser.setDefaultOption("4 Ball", "4 Ball");
      autonChooser.addOption("3 Ball - Human Player", "3 Ball - Human Player");
      autonChooser.addOption("5 Ball", "5 Ball");
      SmartDashboard.putData("Auton Sequence", autonChooser);
    }else if(Constants.Field.MID_POS.equals(initialPosChooser.getSelected()) && !prevPos.equals(Constants.Field.MID_POS)){
      autonChooser  = new SendableChooser<>();
      autonChooser.setDefaultOption("2 Ball", "2 Ball");
      SmartDashboard.putData("Auton Sequence", autonChooser);
    }else if(Constants.Field.LEFT_POS.equals(initialPosChooser.getSelected()) && !prevPos.equals(Constants.Field.LEFT_POS)){
      autonChooser  = new SendableChooser<>();
      autonChooser.setDefaultOption("3 Ball", "3 Ball");
      autonChooser.addOption("2 Ball", "2 Ball");
      SmartDashboard.putData("Auton Sequence", autonChooser);
    }else if(Constants.Field.ZERO.equals(initialPosChooser.getSelected()) && !prevPos.equals(Constants.Field.ZERO)){
      autonChooser  = new SendableChooser<>();
      autonChooser.setDefaultOption("2 Ball", "2 Ball");
      SmartDashboard.putData("Auton Sequence", autonChooser);
    }
    prevPos = initialPosChooser.getSelected();
    // This method will be called once per scheduler run
  }
}
