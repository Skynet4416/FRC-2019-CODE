/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;

public class MakeProfile extends Command 
{
    // key used to make the pi generate a motion profile.
    public static final String REQUEST_KEY = "captureProfiles";
    public static final String[] LEFT_KEYS = { "leftProfile_0", "leftProfile_1", "leftProfile_2" };
    public static final String[] RIGHT_KEYS = { "rightProfile_0", "rightProfile_1", "rightProfile_2" };
    public static final int ARRAY_LEN = 100;
    private NetworkTableEntry _working;
    private NetworkTable _visionTable;

    public MakeProfile()
    {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        _visionTable = inst.getTable("Vision");
        _working = _visionTable.getEntry(REQUEST_KEY);
    }

    private static double[][] zipKeys(String[] keys, NetworkTable table)
    {
        double[][] zipped = new double[keys.length][ARRAY_LEN];

        for (int i = 0; i < keys.length; i++)
        {
            double[] values = table.getEntry(keys[i]).getDoubleArray((double[]) null);
            for (int j = 0; j < values.length && j < ARRAY_LEN; j++)
                zipped[i][j] = values[j];
        }
        return zipped;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        _working.setBoolean(true);
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
        return _working.getBoolean(false); // never happens
    }

    // Called once after isFinished returns true
    @Override
    protected void end()
    {
        double[][] left = zipKeys(LEFT_KEYS, _visionTable);
        double[][] right = zipKeys(RIGHT_KEYS, _visionTable);
        Command follow = new FollowPath(left, right);
        follow.start();
        while (follow.isRunning())
            ;
        follow.close();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted()
    {
        
    }
}
