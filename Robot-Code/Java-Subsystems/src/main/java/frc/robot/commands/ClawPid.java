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
import frc.robot.subsystems.Claw;

public class ClawPid extends Command
{
    private Claw _claw;
    private double _setpoint;
    public static final double TIMEOUT = 2;

    public ClawPid(double setpoint)
    {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.claw);
        this._setpoint = setpoint;
        this._claw = Robot.claw;
    }

    // Called once when the command executes
    @Override
    protected void initialize()
    {
        this._claw.setSetpoint(this._setpoint);
        this._claw.enable();
        SmartDashboard.putBoolean("ClawPID", true);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {

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
        this._claw.disable();
        SmartDashboard.putBoolean("ClawPID", false);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted()
    {
        this._claw.disable();
        SmartDashboard.putBoolean("ClawPID", false);
    }
}
