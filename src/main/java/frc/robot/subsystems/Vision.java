package frc.robot.subsystems;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Uses the CameraServer class to automatically capture video from a USB webcam and send it to the
 * FRC dashboard without doing any vision processing. This is the easiest way to get camera images
 * to the dashboard. Just add this to the robotInit() method in your program.
 */
public class Vision extends SubsystemBase {
    PowerDistribution pdp = new PowerDistribution();
    public Vision() {
        CameraServer.startAutomaticCapture(0).setResolution(32, 32);
        setLED(false);
        CameraServer.startAutomaticCapture(1).setResolution(32, 32);
    }


    public void setLED(boolean isOn){
        pdp.setSwitchableChannel(isOn);
    }


}
