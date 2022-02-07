// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveTrain{

        public static final int RFMOTOR_ID = 1;
        public static final int RBMOTOR_ID = 2;
        public static final int LFMOTOR_ID = 4;
        public static final int LBMOTOR_ID = 3;
        public static final double MAX_VELOCITY = 1.5;
        public static final double MAX_ROT_VELOCITY = 3;
        public static final double MAX_ACCELERATION = 2;
        public static final double TRACK_WIDTH = 0.381;
        public static final double LEFT_kV = 2.1875;
        public static final double LEFT_kS = 0.077369;
        public static final double LEFT_BACKWARD_kV = 1.955;
        public static final double LEFT_BACKWARD_kS = 0.077369;
        public static final double RIGHT_kV = 2.1055;
        public static final double RIGHT_kS = 0.077369;
        public static final double WHEEL_CIRCUMFERENCE = 0.0762 * Math.PI *  2 * 3.323/127/0.363*2.7 * (1.42/1.12);
        public static final Pose2d INITIAL_POS = new Pose2d(0,0, new Rotation2d(0));
        public static final List<Translation2d> AUTON_MIDWAY_POINTS = new ArrayList<Translation2d>();
        public static final Pose2d AUTON_FINAL_POS = new Pose2d(1,1,new Rotation2d(Math.PI/2));
        public static final double kP = 0.001;
        public static final double kI = 0;
        public static final double kD = 0;

    }
    public static final class Controller{

        public static final int PORT = 1;
        public static final int LV_AXIS = 1;
        public static final int RH_AXIS = 3;

    }

    public static final class Joystick{

        public static final int PORT = 2;

    }

    public static final class Shooter{
        public static final int SECOND_PISTON_FORWARD = 0;
        public static final int FIRST_PISTON_FORWARD = 0;
        public static final int FIRST_PISTON_BACKWARD = 0;
        public static final int SECOND_PISTON_BACKWARD = 0;

    }
    public static Robot TimedRobot;
}
