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
            public static final int MASTER_LEFT = 6;
            public static final int MASTER_RIGHT = 1;
            public static final int SLAVE_LEFT = 3;
            public static final int SLAVE_RIGHT = 2;
        }

        public class Claw
        {
            public static final int MOTOR = 9;
        }

        public class Roller
        {
            public static final int LEFT = 7;
            public static final int RIGHT = 8;
        }

        public class Climber
        {
            public static final int COMPRESSOR = 0;
            public static final int FORWARD_SOLENOID1 = 4;
            public static final int REVERSE_SOLENOID1 = 5;
            public static final int FORWARD_SOLENOID2 = 6;
            public static final int REVERSE_SOLENOID2 = 7;
            public static final int PCM_ID = 42;
        }
    }

    /**
     * A class containing input device ports
     */
    public class Controls
    {
        public class Chassis
        {
            public static final int LEFT_JOY = 1;
            public static final int RIGHT_JOY = 2;
        }

        public class Claw
        {
            public static final int CLOSE_BTN = 5;
            public static final int HATCH_BTN = 6;
        }

        public class Roller
        {
            public static final int PUSH_VAL = -1;
            public static final int PULL_VAL = 1;
        }

        public class Elevator
        {
            public static final int ELEVATOR_CONTROL = 0;
            public static final int BUTTOM_BTN = 1;
        }

        public class Climber
        {
            public static final int PNEU_TGL_BTN = 7;
        }
    }

    public class Sensors
    {
        public class Chassis
        {

        }
        
        public class Claw
        {
            public static final int A_CHANNEL = 1;
            public static final int B_CHANNEL = 0;
            public static final int SWITCH = 6;
        }
    }
}
