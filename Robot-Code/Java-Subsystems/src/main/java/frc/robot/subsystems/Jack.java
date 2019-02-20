/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 Jack subsystem
 */
public class Jack extends Subsystem
{
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    // Angle presets
    private static final double ANGLE_MOD_1 = 90;
    private static final double ANGLE_MOD_2 = 0;
    // Left and right servos
    private Servo _leftServo = new Servo(RobotMap.Motors.Jack.LEFT_SERVO);
    private Servo _rightServo = new Servo(RobotMap.Motors.Jack.RIGHT_SERVO);
    // Are the servos on ANGLE_MOD_1 or ANGLE_MOD_2
    private boolean isOpen = false;

    // Toggle the state of the jack and gives it a correct angle
    public void toggle()
    {
        isOpen = !isOpen; // Toggles
        if (isOpen)  // Should it be open now?
        {
            // Angle turns to 90
            this._leftServo.setAngle(ANGLE_MOD_1);
            this._rightServo.setAngle(ANGLE_MOD_1);
        }
        else
        {
            // Angle turns to 0
            this._leftServo.setAngle(ANGLE_MOD_2);
            this._rightServo.setAngle(ANGLE_MOD_2);
        }
    }

    // Sets default command
    @Override
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
    }
}
