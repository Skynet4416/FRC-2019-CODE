/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Claw;

public class ClawPid extends Command
{
    private Claw _claw;
    private Claw.State _clawState;
    public static final double MAX_CURRENT = 1;

    public ClawPid()
    {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.claw);
        this._claw = Robot.claw;
    }

    // Called once when the command executes
    @Override
    protected void initialize()
    {
        _claw.reset();
        _claw.setSetpoint(0);
        this._clawState = Robot.oi.getClaw();
        this._claw.set(this._clawState);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {
        SmartDashboard.putNumber("Claw Count", _claw.getEncoder());
        SmartDashboard.putNumber("Claw Current", Robot.pdp.getCurrent(RobotMap.Motors.Claw.MOTOR));
        /*if (Robot.pdp.getCurrent(RobotMap.Motors.Claw.MOTOR) > MAX_CURRENT)
        {
            _claw.set(0);
            _clawState = Claw.State.none;
        }
        else */
        double stickVal = Robot.oi.getClawJoy();
        SmartDashboard.putNumber("Stick value", stickVal);
        //Positive closes
        if (-stickVal > 0.2)
        {
            SmartDashboard.putBoolean("ClawSwitch", false);
            
            _claw.set(Robot.oi.getClawJoy());
            _clawState = Claw.State.none;
        }
        else if (this._claw.getSwitch())
        {
            SmartDashboard.putBoolean("ClawSwitch", true);
            this._claw.set(0);
            this._claw.setZero();
        }
        else if (stickVal > 0.2)
        {
            SmartDashboard.putBoolean("ClawSwitch", false);
            
            _claw.set(Robot.oi.getClawJoy());
            _clawState = Claw.State.none;
        }
        else if (_clawState != Robot.oi.getClaw())
        {
            SmartDashboard.putBoolean("ClawSwitch", false);
            
            _clawState = Robot.oi.getClaw();
            _claw.set(_clawState);
        }
        else if (_clawState == Claw.State.none)
        {
            SmartDashboard.putBoolean("ClawSwitch", false);
            
            _claw.set(0);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished()
    {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end()
    {
        this._claw.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted()
    {
        this._claw.disable();
    }
}
