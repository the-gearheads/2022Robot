// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("Shooting");
  private boolean flag = true;
  /** Creates a new Vision. */
  public Vision() {}

  @Override
  public void periodic() {
    flag = !flag;
    SmartDashboard.putBoolean("Flag", flag);
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      List<PhotonTrackedTarget> targets = result.getTargets();

      double meanX = targets.stream().map((e)->e.getYaw()).reduce(0.0, Double::sum);
      meanX/=targets.size();
      SmartDashboard.putNumber("Mean X", meanX);
    }
    // This method will be called once per scheduler run
  }
}
