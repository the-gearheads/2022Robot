package frc.robot.subsystems;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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
    public boolean isRunning = false;
    public boolean isOn = false;
    AddressableLED ledStrip = new AddressableLED(9);
    AddressableLEDBuffer greenBuffer = new AddressableLEDBuffer(27);
    AddressableLEDBuffer disabledBuffer = new AddressableLEDBuffer(27);

    public Vision() {
        ledStrip.setLength(27);

        // for loop to fill entire buffer
        for (int i = 0; i < greenBuffer.getLength(); i++)
        {
            greenBuffer.setRGB(i, 0, 255, 0);
        }

        for (int i = 0; i < disabledBuffer.getLength(); i++)
        {
            disabledBuffer.setRGB(i, 0, 0, 0);
        }
    }
    public void turnOn() {
        ledStrip.setData(greenBuffer);
        ledStrip.start();
        isOn = true;
    }

    public void turnOff() {
        ledStrip.setData(disabledBuffer);
        ledStrip.start();
        isOn = false;
    }
}
