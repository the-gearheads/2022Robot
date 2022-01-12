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

        public static final int RFMOTOR_ID = 0;
        public static final int RBMOTOR_ID = 0;
        public static final int LFMOTOR_ID = 0;
        public static final int LBMOTOR_ID = 0;
        public static final double LEFT_kP = 0;
        public static final double LEFT_kI = 0;
        public static final double LEFT_kD = 0;
        public static final double MAX_VELOCITY = 0;
        public static final double MAX_ACCELERATION = 0;
        public static final double RIGHT_kP = 0;
        public static final double RIGHT_kI = 0;
        public static final double RIGHT_kD = 0;
        public static final double TRACK_WIDTH = 0;
        public static final double kV = 0;
        public static final double kS = 0;
        public static final int WHEEL_RADIUS = 0;
        public static final Pose2d INITIAL_POS = new Pose2d(0,0, new Rotation2d(0));
        public static final List<Translation2d> AUTON_MIDWAY_POINTS = new ArrayList<Translation2d>();
        public static final Pose2d AUTON_FINAL_POS = new Pose2d(0,0, new Rotation2d(0));

    }
    public static final class Controller{
        
    }
}
