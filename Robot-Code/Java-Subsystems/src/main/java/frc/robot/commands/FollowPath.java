/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class FollowPath extends Command 
{
    public static final double WHEEL_DIAMETER = 2;  // Wheel diameter in meters
    public static final double PERIMETER = WHEEL_DIAMETER * Math.PI;  // Wheel perimeter
    public static final int MIN_POINTS = 10;  // Minimum amount of points to be loaded to the talon at once
    private TalonSRX _talon;  // The talon
    private BufferedTrajectoryPointStream _buffer;  // Points buffer
    private double [][] _points;  // Points

    public FollowPath(double [][] points, TalonSRX talon)
    {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.chassis);
        this._talon = talon;
        this._buffer = new BufferedTrajectoryPointStream();
        // Copying points
        this._points = new double [points.length][points[0].length];
        for (int pointIndex = 0; pointIndex < this._points.length; pointIndex++)
        {
            for (int i = 0; i < this._points[pointIndex].length; i++)
            {
                this._points[pointIndex][i] = points[pointIndex][i];
            }
        }   
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        TrajectoryPoint point = new TrajectoryPoint();
        point.profileSlotSelect0 = 0;
        this._buffer.Clear();
        for (int j = 0; j < this._points.length; j++)
        {
            //Meters --> Rotations
            point.position = this._points[j][0] / PERIMETER;
            //Meters / second --> Rotations / minute
            point.velocity = this._points[j][1] / (PERIMETER / 60);
            //Seconds --> Milliseconds
            point.timeDur = (int)(this._points[j][2] / 1000);
            point.zeroPos = j == 0;
            point.isLastPoint = j + 1 == this._points.length;
            this._buffer.Write(point);
        }
        // Stars the motion profiling
        this._talon.startMotionProfile(this._buffer, MIN_POINTS, ControlMode.MotionProfile);
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
        return this._talon.isMotionProfileFinished();
    }

    // Called once after isFinished returns true
    @Override
    protected void end()
    {

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted()
    {
        
    }
}