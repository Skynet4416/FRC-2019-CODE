/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.*;
import frc.robot.commands.*;
import frc.robot.subsystems.Claw;

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
    private XboxController _systemsXbox = new XboxController(RobotMap.Controls.Elevator.ELEVATOR_CONTROL);
    // Toggle climbing
    private Button _climbingTgl = new JoystickButton(_systemsXbox, RobotMap.Controls.Climber.PNEU_TGL_BTN);
    // Toggle jack
    private Button _openJack = new JoystickButton(_systemsXbox, RobotMap.Controls.Jack.TGL_JACK);
    
    private Button _clawPanelButton = new JoystickButton(_leftJoy, 11);
    public static enum ButtonStatus {pressed, released, none};

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
        return this._systemsXbox.getY(Hand.kLeft);
        //return 0;
    }

    public double getRoller()
    {
        return this._systemsXbox.getTriggerAxis(Hand.kRight) - this._systemsXbox.getTriggerAxis(Hand.kLeft);
    }

    public boolean getElevX()
    {
        return _systemsXbox.getXButton();
    }

    public Claw.State getClaw()
    {
        if (getRoller() < -0.5)
        {
            return Claw.State.cargo;
        }
        if (_systemsXbox.getRawButton(RobotMap.Controls.Claw.CLOSE_BTN))
        {
            return Claw.State.closed;
        }
        if (_systemsXbox.getRawButton(RobotMap.Controls.Claw.HATCH_BTN))
        {
            return Claw.State.panel;
        }
        return Claw.State.none;
    }

    public double getClawJoy()
    {
        return _systemsXbox.getY(Hand.kRight);
    }

    public boolean getJack()
    {
        return _systemsXbox.getRawButton(RobotMap.Controls.Jack.TGL_JACK);
    }

    public boolean getButtom()
    {
        return false; // _systemsXbox.getRawButton(RobotMap.Controls.Elevator.BUTTOM_BTN);
    }

    public ButtonStatus getClimber()
    {
        if (_systemsXbox.getRawButtonPressed(RobotMap.Controls.Climber.PNEU_TGL_BTN))
        {
            return ButtonStatus.pressed;
        }
        if (_systemsXbox.getRawButtonReleased(RobotMap.Controls.Climber.PNEU_TGL_BTN))
        {
            return ButtonStatus.released;
        }
        return ButtonStatus.none;
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
