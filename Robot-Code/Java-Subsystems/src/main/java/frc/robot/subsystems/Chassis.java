/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.DriveByJoy;

/**
 * Add your docs here.
 */
public class Chassis extends Subsystem
{
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private TalonSRX _rightMaster = new TalonSRX(RobotMap.Motors.Chassis.MASTER_RIGHT);
    private TalonSRX _rightSlave = new TalonSRX(RobotMap.Motors.Chassis.SLAVE_RIGHT);
    private TalonSRX _leftMaster = new TalonSRX(RobotMap.Motors.Chassis.MASTER_LEFT);
    private TalonSRX _leftSlave = new TalonSRX(RobotMap.Motors.Chassis.SLAVE_LEFT);

    public Chassis()
    {// sets the slaves to follow the masters
        this._rightSlave.set(ControlMode.Follower, this._rightMaster.getDeviceID());
        this._leftSlave.set(ControlMode.Follower, this._leftMaster.getDeviceID());
    }

    public void set(double left, double right)
    {
        if (Math.abs(left) > 1 || Math.abs(right) > 1)
        {// if invalid value is passed
            System.out.println("Chassis: invalid value recieved to drive");
            return;
        }
        this._rightMaster.set(ControlMode.PercentOutput, right);
        this._leftMaster.set(ControlMode.PercentOutput, left);
    }

    @Override
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        setDefaultCommand(new DriveByJoy());
    }
}
