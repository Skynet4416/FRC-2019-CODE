
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorByJoy;

/**
 * Elevator subsystem
 */
public class Elevator extends Subsystem 
{
    private TalonSRX _master = new TalonSRX(RobotMap.Motors.Elevator.MASTER); // Talon1
    private TalonSRX _slave = new TalonSRX(RobotMap.Motors.Elevator.SLAVE); // Talon2

    public Elevator()
    {
        _slave.set(ControlMode.Follower, _master.getBaseID());
        _master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    }
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
        this._master.set(ControlMode.PercentOutput, -power);
        SmartDashboard.putNumber("Elev Count", this.getEncoder());
    }
    public double getEncoder()
    {
        return _master.getSelectedSensorPosition();
    }

    // Sets default command
    @Override
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        setDefaultCommand(new ElevatorByJoy());
    }
}