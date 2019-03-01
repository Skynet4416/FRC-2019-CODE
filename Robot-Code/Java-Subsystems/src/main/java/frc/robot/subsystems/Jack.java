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
    // Left and right servos
    private Servo _leftServo = new Servo(RobotMap.Motors.Jack.LEFT_SERVO);
    private Servo _rightServo = new Servo(RobotMap.Motors.Jack.RIGHT_SERVO);

    public Jack()
    {
    }

    // Toggle the state of the jack and gives it a correct angle
    public void open()
    {
        this._leftServo.set(1);
        this._rightServo.set(0);
    }

    public void close()
    {
        this._leftServo.set(0.6);
        this._rightServo.set(0.4);
    }

    public String getAngles()
    {
        return _leftServo.getAngle() + ", " + _rightServo.getAngle();
    }

    // Sets default command
    @Override
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        
    }
}
