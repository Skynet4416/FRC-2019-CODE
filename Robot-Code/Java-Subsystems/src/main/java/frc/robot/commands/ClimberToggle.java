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
import frc.robot.subsystems.Climber;

public class ClimberToggle extends Command
{
    private Climber _climber;
    public static final double TIMEOUT = 1;

    public ClimberToggle()
    {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.climber);
        this._climber = Robot.climber;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        setTimeout(TIMEOUT);
        SmartDashboard.putBoolean("Pneu", true);
        this._climber.toggleSolenoid();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {
        // If can test limit with compressor or something
        // Check will be here
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
        SmartDashboard.putBoolean("Pneu", false);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted()
    {
        // stopped by interupter
        SmartDashboard.putBoolean("Pneu", false);
    }
}
