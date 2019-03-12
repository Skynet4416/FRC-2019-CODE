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

public class ElevatorByJoy extends Command 
{
    private Elevator _elevator;
    private boolean _isPID;
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
        _isPID = false;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {
        SmartDashboard.putNumber("elev_joy", Robot.oi.getElevator());
        if (Robot.oi.getElevX())
        {
            _isPID = true;
            _elevator.goToHatch();
            return;
        }
        if (_isPID && Math.abs(Robot.oi.getElevator()) < 0.2)
        {
            return;
        }
        _isPID = false;
        this._elevator.set(Robot.oi.getElevator() + ZERO_VALUE);
        if(!this._elevator.getSwitch())
        {
            this._elevator.setZero();
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