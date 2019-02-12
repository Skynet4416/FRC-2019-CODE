
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorByJoy;

public class Elevator extends Subsystem 
{
    private TalonSRX _motor1 = new TalonSRX(RobotMap.Motors.Elevator.MOTOR1);
    private TalonSRX _motor2 = new TalonSRX(RobotMap.Motors.Elevator.MOTOR2);

    
    public void set(double power)
    {
        if (Math.abs(power) > 1)
        {
            //unepic println
            System.out.println("Elevator: invalid value recieved to drive - Cropping");
            power /= Math.abs(power);  // Makes it either 1 or -1 (basically copysign of power to 1)
        }
        //Not sure what direction the output will be but kalisch didnt answer so...
        //Positive power goes up and negative goes down (Wanted outcome)
        this._motor1.set(ControlMode.PercentOutput, power);
        this._motor2.set(ControlMode.PercentOutput, power);
    }
    @Override
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        setDefaultCommand(new ElevatorByJoy());
    }
}