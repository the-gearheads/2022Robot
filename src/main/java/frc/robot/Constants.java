// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auton.AutonDrive;

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

        public static final int TEST_RFMOTOR_ID = 1;
        public static final int TEST_RBMOTOR_ID = 2;
        public static final int TEST_LFMOTOR_ID = 3;
        public static final int TEST_LBMOTOR_ID = 4;


        public static final int RFMOTOR_ID = 4;
        public static final int RBMOTOR_ID = 5;
        public static final int LFMOTOR_ID = 6;
        public static final int LBMOTOR_ID = 7;
        public static double MAX_VELOCITY = 3;//1
        public static final double MAX_ROT_VELOCITY = 2.5;//2.5
        public static double MAX_ACCELERATION = 6;
        public static final double TEST_TRACK_WIDTH = 0.381;
        public static final double TRACK_WIDTH = 0.622;
        public static final double LEFT_kV = 2.977147245265187;//2.11//3.249
        public static final double LEFT_kS = 0.5;//0.077369
        public static final double LEFT_BACKWARD_kV = 3.248834213489218;//2.12
        public static final double LEFT_BACKWARD_kS = 0.077369;
        public static final double RIGHT_kV =    2.973787777757068;//2.1055//3.248
        public static final double RIGHT_kS = 0.5;//0.077369
        
        public static final double TEST_RIGHT_kV =   1.6236801243419694;//2.19
        public static final double TEST_RIGHT_kS = 0;
        public static final double TEST_LEFT_kV =   1.6236801243419694;//2.19
        public static final double TEST_LEFT_kS = 0;
        public static final double TEST_LEFT_BACKWARD_kV =    1.6236801243419694;//2.19
        public static final double TEST_LEFT_BACKWARD_kS = 0;
        public static final double WHEEL_CIRCUMFERENCE = 0.2032 * Math.PI;//used to be 0.19

        public static final double TEST_WHEEL_CIRCUMFERENCE =0.15*Math.PI;
        public static final Pose2d INITIAL_POS = new Pose2d(0,0, new Rotation2d(0));
        public static final List<Translation2d> AUTON_MIDWAY_POINTS = new ArrayList<Translation2d>();
        public static final Pose2d AUTON_FINAL_POS = new Pose2d(1,1,new Rotation2d(Math.PI/2));
        public static final double kP = 0.001;
        public static final double kI = 0;
        public static final double kD = 0;
        public static double FORWARD_DIRECTION = 1;
        public static double LEFT_SPEED_CONSTANT = 0.97;

    }
    public static final class Controller{

        public static final int PORT = 1;
        public static final int LV_AXIS = 1;
        public static final int RH_AXIS = 4;

    }

    public static final class Joystick{

        public static final int PORT = 3;

    }

    public static final class Shooter{
        public static final int RETRACT_LEFT_SOLENOID = 0;
        public static final int EXTEND_LEFT_SOLENOID = 1;
        public static final int EXTEND_RIGHT_SOLENOID = 2;
    }

    public static final class Field{

        public static final Pose2d INIT_POS_3 = new Pose2d(7.522,2.196,new Rotation2d(-1.95461395008));
        public static Pose2d INIT_POS_1 = new Pose2d(7.9,3,new Rotation2d(-1.95461395008));
        public static Pose2d INIT_POS_2 = new Pose2d(0,0,new Rotation2d(0));
        
        public static final Pose2d RIGHT_POS = new Pose2d(7.3,1.5,new Rotation2d(-1.5707963267948966)); //new Pose2d(7.3,1.5,new Rotation2d(1.5707963267948966))
        public static final Pose2d RIGHT_SHOOT_POS = new Pose2d(7.5,2.3,new Rotation2d(-1.9513027039072617)); //new Pose2d(7.3,1.5,new Rotation2d(1.5707963267948966))
        public static final Pose2d RIGHT_4_BALL = new Pose2d(6.462016556751126,2.3130872353349536,new Rotation2d(-2.287338000891296)); //new Pose2d(7.3,1.5,new Rotation2d(1.5707963267948966))
        public static final Pose2d MID_POS = new Pose2d(1,1,new Rotation2d(1));
        public static final Pose2d LEFT_POS = new Pose2d(6.0822469643424935,5.3780426654553715,new Rotation2d( 2.5535900500422266));
        public static final Pose2d ZERO = new Pose2d(0,0,new Rotation2d(0));
    }
    public static final class Climber{
        public static final int LEFT_ID  = 11;
        public static final int RIGHT_ID = 12;
        public static final int LEFT_LIMIT = 3;
        public static final int RIGHT_LIMIT = 2;
        public static final double MAX_SPEED = 0;
        public static final double MIN_SPEED = 0;

    }

    public static final class LightSensor{
        public static final int PORT = 1;
    }

    public static Robot TimedRobot;

    public static final class ColorSensor{
        public static final I2C.Port PORT = I2C.Port.kOnboard;
    }

    public static final class Intake{
        public static final int EXTEND_SOLENOID = 4;
        public static final int RETRACT_SOLENOID = 3;
        public static final int LEFT_MOTOR = 10;
        public static final int RIGHT_MOTOR = 9;

    }

    public static final class Elevator{
        public static final int MOTOR_ID = 8;
        public static boolean shot = false;
        public static boolean color_status = false;
        public static boolean auto = false;
    }
}
