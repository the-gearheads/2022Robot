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
  public double xHuman = 1.835;
  public double yHuman = 0.56;
  public double rotHuman = -2.2;

  public double xShoot = 7.03;
  public double yShoot = 1.695;
  public double rotShoot = -1.82;

  public AutonChooser() {
    initialPosChooser.setDefaultOption("Right", Constants.Field.RIGHT_POS);
    initialPosChooser.addOption("Mid", Constants.Field.MID_POS);
    initialPosChooser.addOption("Left", Constants.Field.LEFT_POS);
    initialPosChooser.addOption("Generic", Constants.Field.ZERO);

    SmartDashboard.putData("Initial Position", initialPosChooser);
    SmartDashboard.putBoolean("4 Ball?", true);

    SmartDashboard.putNumber("5 Ball Human X",xHuman);
    SmartDashboard.putNumber("5 Ball Human Y", yHuman);
    SmartDashboard.putNumber("5 Ball Human Rot", rotHuman);


    SmartDashboard.putNumber("5 Ball Shoot X", xShoot);
    SmartDashboard.putNumber("5 Ball Shoot Y", yShoot);
    SmartDashboard.putNumber("5 Ball Shoot Rot", rotShoot);

    SmartDashboard.putNumber("Velocity", 4);
    SmartDashboard.putNumber("Acc", 1.2);
  }

  @Override
  public void periodic() {
    if(Constants.Field.RIGHT_POS.equals(initialPosChooser.getSelected()) && !prevPos.equals(Constants.Field.RIGHT_POS)){
      autonChooser  = new SendableChooser<>();
      // autonChooser.setDefaultOption("5 Ball", "5 Ball");
      autonChooser.setDefaultOption("4 Ball", "4 Ball");
      // autonChooser.addOption("3 Ball - Human Player", "3 Ball - Human Player");
      SmartDashboard.putData("Auton Sequence", autonChooser);
    }else if(Constants.Field.MID_POS.equals(initialPosChooser.getSelected()) && !prevPos.equals(Constants.Field.MID_POS)){
      autonChooser  = new SendableChooser<>();
      SmartDashboard.putData("Auton Sequence", autonChooser);
    }else if(Constants.Field.LEFT_POS.equals(initialPosChooser.getSelected()) && !prevPos.equals(Constants.Field.LEFT_POS)){
      autonChooser  = new SendableChooser<>();
      autonChooser.setDefaultOption("3 Ball", "3 Ball");
      SmartDashboard.putData("Auton Sequence", autonChooser);
    }else if(Constants.Field.ZERO.equals(initialPosChooser.getSelected()) && !prevPos.equals(Constants.Field.ZERO)){
      autonChooser  = new SendableChooser<>();
      autonChooser.setDefaultOption("2 Ball", "2 Ball");
      SmartDashboard.putData("Auton Sequence", autonChooser);
    }
    prevPos = initialPosChooser.getSelected();
    // This method will be called once per scheduler run


    if(SmartDashboard.getNumber("5 Ball Human X",0) == 0){
      SmartDashboard.putNumber("5 Ball Human X",xHuman);
    }
    if(SmartDashboard.getNumber("5 Ball Human Y", 0) == 0){
      SmartDashboard.putNumber("5 Ball Human Y", yHuman);
    }    
    if(SmartDashboard.getNumber("5 Ball Human Rot", 0) == 0){
      SmartDashboard.putNumber("5 Ball Human Rot", rotHuman);
    }


    if(SmartDashboard.getNumber("5 Ball Shoot X", 0) == 0){
      SmartDashboard.putNumber("5 Ball Shoot X", xShoot);
    }
    if(SmartDashboard.getNumber("5 Ball Shoot Y", 0) == 0){
      SmartDashboard.putNumber("5 Ball Shoot Y", yShoot);
    }
    if(SmartDashboard.getNumber("5 Ball Shoot Rot", 0) == 0){
    SmartDashboard.putNumber("5 Ball Shoot Rot", rotShoot);
    }

    if(SmartDashboard.getNumber("Velocity", 0) == 0){
      SmartDashboard.putNumber("Velocity", 4);
    }
    if(SmartDashboard.getNumber("Acc", 0) == 0){
      SmartDashboard.putNumber("Acc", 1.2);
    }
  }
}
