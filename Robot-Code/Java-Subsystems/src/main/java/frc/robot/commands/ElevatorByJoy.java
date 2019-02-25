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

public class ElevatorByJoy extends Command 
{
    private Elevator _elevator;
    public static final double ZERO_VALUE = 0;  // Value in which the elevator will stay put

    public ElevatorByJoy()
    {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.elevator);
        this._elevator = Robot.elevator;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize()
    {

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {
        this._elevator.set(Robot.oi.getElevator() + ZERO_VALUE);
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

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    //When interrupted - Elevator gets stuck
    @Override
    protected void interrupted()
    {
        this._elevator.set(ZERO_VALUE);
    }
}