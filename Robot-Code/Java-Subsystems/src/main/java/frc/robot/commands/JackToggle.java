/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.*;

/*
 Jack toggler basically
*/
public class JackToggle extends Command 
{
    private Jack _jack;
    public final double TIMEOUT = 0.3;

    public JackToggle()
    {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.jack);
        this._jack = Robot.jack;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        this._jack.open();
        this.setTimeout(TIMEOUT);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {
        System.out.println(this._jack.getAngles());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished()
    {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    @Override
    protected void end()
    {
        //this._jack.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted()
    {
        //this._jack.stop();
    }
}
