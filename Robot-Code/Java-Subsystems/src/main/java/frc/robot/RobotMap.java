/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap
{
    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.
    // public static int leftMotor = 1;
    // public static int rightMotor = 2;

    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static int rangefinderPort = 1;
    // public static int rangefinderModule = 1;
    /**
     * A class containing motor ports
     */
    public class Motors
    {
        public class Chassis
        {
            public static final int MASTER_LEFT = 0;
            public static final int MASTER_RIGHT = 0;
            public static final int SLAVE_LEFT = 0;
            public static final int SLAVE_RIGHT = 0;
        }

        public class Claw
        {
            public static final int MOTOR = 0;
        }

        public class Roller
        {
            public static final int LEFT = 0;
            public static final int RIGHT = 0;
        }

        public class Elevator
        {
            public static final int MOTOR1 = 0;
            public static final int MOTOR2 = 0;
        }
    }

    /**
     * A class containing input device ports
     */
    public class Controls
    {
        public class Chassis
        {
            public static final int LEFT_JOY = 0;
            public static final int RIGHT_JOY = 0;
        }

        public class Claw
        {
            public static final int OPEN_BTN = 0;
            public static final int CLOSE_BTN = 0;
        }

        public class Roller
        {
            public static final int PUSH_BTN = 0;
            public static final int PULL_BTN = 0;
        }

        public class Elevator
        {
            public static final int ELEVATOR_JOY = 0;
        }
    }
}
