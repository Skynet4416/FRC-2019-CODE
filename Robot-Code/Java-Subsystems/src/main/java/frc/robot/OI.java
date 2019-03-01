/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
import frc.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI
{
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a
    //// joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.
    //  Chassis joysticks
    private Joystick _leftJoy = new Joystick(RobotMap.Controls.Chassis.LEFT_JOY);
    private Joystick _rightJoy = new Joystick(RobotMap.Controls.Chassis.RIGHT_JOY);
    // Elevator joystick
    private Joystick _elevatorJoy = new Joystick(RobotMap.Controls.Elevator.ELEVATOR_JOY);
    // Roller gripper buttons
    private Button _pushBtn = new JoystickButton(_elevatorJoy, RobotMap.Controls.Roller.PUSH_BTN);
    private Button _pullBtn = new JoystickButton(_elevatorJoy, RobotMap.Controls.Roller.PULL_BTN);
    // Claw buttons
    private Button _openBtn = new JoystickButton(_elevatorJoy, RobotMap.Controls.Claw.OPEN_BTN);
    private Button _closeBtn = new JoystickButton(_elevatorJoy, RobotMap.Controls.Claw.CLOSE_BTN);
    // Toggle climbing
    private Button _climbingTgl = new JoystickButton(_elevatorJoy, RobotMap.Controls.Climber.PNEU_TGL_BTN);
    // Toggle jack
    private Button _openJack = new JoystickButton(_elevatorJoy, RobotMap.Controls.Jack.TGL_JACK);
    
    private Button _clawTestButton = new JoystickButton(_leftJoy, 1);
    //private Button _clawCloseButton = new JoystickButton(_leftJoy, 3);
    //private Button _clawCargoButton = new JoystickButton(_leftJoy, 4);
    //private Button _clawPanelButton = new JoystickButton(_leftJoy, 5);
    
    public OI()
    {
        this._pushBtn.whenPressed(new RollerPush());
        this._pushBtn.whenReleased(new RollerStop());
        this._pullBtn.whenPressed(new RollerPull());
        this._pullBtn.whenReleased(new RollerStop());
        this._openBtn.whenPressed(new ClawOpen());
        this._openBtn.whenReleased(new ClawStop());
        this._closeBtn.whenPressed(new ClawClose());
        this._closeBtn.whenReleased(new ClawStop());
        this._climbingTgl.whenPressed(new ClimberToggle());
        this._openJack.whenPressed(new JackToggle());
    }

    public double getLeft()
    {
        return this._leftJoy.getY();
        //return 0;
    }

    public double getRight()
    {
        return this._rightJoy.getY();
        //return 0;
    }

    public double getElevator()
    {
        return this._elevatorJoy.getY();
        //return 0;
    }
    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());
}
