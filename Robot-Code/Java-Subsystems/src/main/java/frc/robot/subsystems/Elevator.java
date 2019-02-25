
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorByJoy;

/**
 * Elevator subsystem
 */
public class Elevator extends Subsystem 
{
    private TalonSRX _motor1 = new TalonSRX(RobotMap.Motors.Elevator.MOTOR1); // Talon1
    private TalonSRX _motor2 = new TalonSRX(RobotMap.Motors.Elevator.MOTOR2); // Talon2

    public void set(double power)
    {
        if (Math.abs(power) > 1)
        {
            // Makes it either 1 or -1 (basically copysign of power to 1)
            power /= Math.abs(power);
            //unepic println
            System.out.println("Elevator: invalid value recieved to drive: Rounding value to: " + power);
        }
        //Positive power goes up and negative goes down (Wanted outcome)
        this._motor1.set(ControlMode.PercentOutput, -power);
        this._motor2.set(ControlMode.PercentOutput, -power);
    }

    // Sets default command
    @Override
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        setDefaultCommand(new ElevatorByJoy());
    }
}