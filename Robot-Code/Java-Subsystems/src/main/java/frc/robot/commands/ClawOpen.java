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
import frc.robot.subsystems.*;

public class ClawOpen extends Command
{
    private Claw _claw;
    public static final double POWER = -0.2;

    public ClawOpen()
    {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.claw);
        this._claw = Robot.claw;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        this._claw.set(POWER);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {
        SmartDashboard.putNumber("Claw Count", _claw.getEncoder());
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
        // stopped by default method
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted()
    {
        // stopped by interupter
    }
}
