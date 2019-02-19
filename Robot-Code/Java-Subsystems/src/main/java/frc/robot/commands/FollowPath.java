/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Chassis;
import frc.robot.Robot;

public class FollowPath extends Command 
{
    public static final double WHEEL_DIAMETER = 2;  // Wheel diameter in meters
    public static final int TALONS = 2;
    public static final double PERIMETER = WHEEL_DIAMETER * Math.PI;  // Wheel perimeter
    private double [][][] _points;  // Points
    private Chassis _chassis;

    public FollowPath(double [][][] points)
    {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.chassis);
        this._chassis = Robot.chassis;
        // Copying points
        this._points = new double [TALONS][points.length][points[0].length];
        for (int talon = 0; talon < this._points.length; talon++)
        {
            for (int pointIndex = 0; pointIndex < this._points[talon].length; pointIndex++)
            {
                for (int i = 0; i < this._points[talon][pointIndex].length; i++)
                {
                    this._points[talon][pointIndex][i] = points[talon][pointIndex][i];
                }
            }
        }   
    }

    private static BufferedTrajectoryPointStream createBuffer(double [][] points)
    {
        TrajectoryPoint point = new TrajectoryPoint();
        point.profileSlotSelect0 = 0;
        BufferedTrajectoryPointStream buffer = new BufferedTrajectoryPointStream();
        for (int j = 0; j < points.length; j++)
        {
            //Meters --> Rotations
            point.position = points[j][0] / PERIMETER;
            //Meters / second --> Rotations / minute
            point.velocity = points[j][1] / (PERIMETER / 60);
            //Seconds --> Milliseconds
            point.timeDur = (int)(points[j][2] / 1000);
            point.zeroPos = j == 0;
            point.isLastPoint = j + 1 == points.length;
            buffer.Write(point);
        }
        return buffer;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        // Creates buffers for each talon track, I guess
        BufferedTrajectoryPointStream [] buffers = new BufferedTrajectoryPointStream[TALONS];
        // Sets
        for (int i = 0; i < this._points.length; i++)
        {
            buffers[i] = createBuffer(this._points[i]);
        }
        // Starts the motion profiling
        this._chassis.setMotionProfiling(buffers);
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
        return this._chassis.isMotionProfileFinished();
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