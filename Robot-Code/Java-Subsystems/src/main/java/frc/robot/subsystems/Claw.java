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
import frc.robot.commands.ClawStop;

/**
 * Add your docs here.
 */
public class Claw extends Subsystem
{
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private TalonSRX _clawMotor = new TalonSRX(RobotMap.Motors.Claw.MOTOR);
    /**
     * Powers the gripper with supplied input
     * 
     * @param power percentage of power to supply to the motors. If invalid,
     *              rounded.
     */
    public void set(double power)
    {
        if (Math.abs(power) > 1)
        {
            // Makes it either 1 or -1 (basically copysign of power to 1)
            power /= Math.abs(power);
            // Still an unepic print message
            System.out.println("Claw: Power input is out of range: Rounding power to: " + power);
        }
        this._clawMotor.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        setDefaultCommand(new ClawStop());
    }
}
