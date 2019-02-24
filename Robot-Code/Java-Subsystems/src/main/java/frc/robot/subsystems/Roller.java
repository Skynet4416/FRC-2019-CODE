/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.RollerStop;

/**
 * Add your docs here.
 */
public class Roller extends Subsystem
{
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private VictorSPX _rightMotor = new VictorSPX(RobotMap.Motors.Roller.RIGHT);
    private VictorSPX _leftMotor = new VictorSPX(RobotMap.Motors.Roller.LEFT);

    
    public Roller()
    {
        this._leftMotor.setInverted(true);
        this._leftMotor.set(ControlMode.Follower, RobotMap.Motors.Roller.RIGHT);
    }
    
    /**
     * Powers the gripper with supplied input
     * 
     * @param power percentage of power to supply to the motors. If invalid,
     *              rounded.
     */
    public void set(double power)
    {
        if (power > 1)
        {// rounds down power over the maximum
            power = 1;
            System.out.println("Roller: rounded down power over 1");
        }
        else if (power < -1)
        {// rounds up power under the minimum
            power = -1;
            System.out.println("Roller: rounded up power under -1");
        }
        // one of the motors needs to rotate opposite the other to make the gripper work
        this._rightMotor.set(ControlMode.PercentOutput, power);
    }

    // Sets default command
    @Override
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        setDefaultCommand(new RollerStop());
    }
}
