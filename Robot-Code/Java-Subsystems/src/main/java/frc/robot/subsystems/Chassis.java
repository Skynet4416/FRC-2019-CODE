/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.DriveByJoy;

/**
 * Chassis subsystem
 */
public class Chassis extends Subsystem
{
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public static final int MIN_POINTS = 10;  // Minimum amount of points to be loaded to the talon at once
    private TalonSRX _rightMaster = new TalonSRX(RobotMap.Motors.Chassis.MASTER_RIGHT);  // Leading right talon
    private TalonSRX _rightSlave = new TalonSRX(RobotMap.Motors.Chassis.SLAVE_RIGHT);  // Following right talon
    private TalonSRX _leftMaster = new TalonSRX(RobotMap.Motors.Chassis.MASTER_LEFT); // Leading left talon
    private TalonSRX _leftSlave = new TalonSRX(RobotMap.Motors.Chassis.SLAVE_LEFT);   // Following left talon

    // Constructor
    public Chassis()
    {
        // sets the slaves to follow the masters
        this._rightSlave.set(ControlMode.Follower, this._rightMaster.getDeviceID());
        this._leftSlave.set(ControlMode.Follower, this._leftMaster.getDeviceID());
        this._rightMaster.setInverted(true);
        this._leftSlave.setInverted(true);
        //configs the encoders
        _rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        _leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    }

    // Controls the chassis in tank drive
    public void set(double left, double right)
    {
        if (Math.abs(left) > 1 || Math.abs(right) > 1)
        {
            // if invalid value is passed
            System.out.println("Chassis: invalid value recieved to drive");
            return;
        }
        this._rightMaster.set(ControlMode.PercentOutput, right);
        this._leftMaster.set(ControlMode.PercentOutput, left);
        this.updateDashboard();
    }

    // Sets motion profiling for the chassis
    public void setMotionProfiling(BufferedTrajectoryPointStream [] buffers)
    {
        // No clue how or why but im checking it just in case
        if (buffers.length != 2)
        {
            System.out.println("Chassis: Somehow the improbable happened.");
            return;
        }
        // Starts the motion profiling
        this._leftMaster.startMotionProfile(buffers[0], MIN_POINTS, ControlMode.MotionProfile);
        this._rightMaster.startMotionProfile(buffers[1], MIN_POINTS, ControlMode.MotionProfile);
    }

    // Returns true if the motion profiling is done
    public boolean isMotionProfileFinished()
    {
        return this._leftMaster.isMotionProfileFinished() || this._rightMaster.isMotionProfileFinished();
    }

    public void updateDashboard()
    {
        SmartDashboard.putNumber("Right Count", _rightMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("Left Count", -1 * _leftMaster.getSelectedSensorPosition());
    }
    
    // Sets default command
    @Override
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        setDefaultCommand(new DriveByJoy());
    }
}
