
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorByJoy;

/**
 * Elevator subsystem
 */
public class Elevator extends PIDSubsystem 
{
    private TalonSRX _master = new TalonSRX(RobotMap.Motors.Elevator.MASTER); // Talon1
    private TalonSRX _slave = new TalonSRX(RobotMap.Motors.Elevator.SLAVE); // Talon2
    private Encoder _encoder = new Encoder(RobotMap.Sensors.Elevator.A_CHANNEL, RobotMap.Sensors.Elevator.B_CHANNEL, false, EncodingType.k4X);
    private DigitalInput _buttomSwitch = new DigitalInput(RobotMap.Sensors.Elevator.SWITCH);
    public static final double TOLERANCE = 15;
	public static final double DISTANCE_PER_TICK = 0.00762 * Math.PI / 2048;

    public Elevator()
    {
        // super(system name, proportional, integral, differential)
      	super("Elevator", SmartDashboard.getNumber("elev_kp", 0), SmartDashboard.getNumber("elev_ki", 0),
            SmartDashboard.getNumber("elev_kd", 0));
        this._slave.set(ControlMode.Follower, this._master.getDeviceID());
        this._encoder.setDistancePerPulse(DISTANCE_PER_TICK);
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
        else if (Math.abs(power) < 0.4)
        {
            power = SmartDashboard.getNumber("Static", 0);
        }
        SmartDashboard.putBoolean("elev_switch", _buttomSwitch.get());
        if (_buttomSwitch.get())
        {
            // negative brings the motor up
            power = Math.min(power, 0);
        }
        //Positive power goes up and negative goes down (Wanted outcome)
        this._master.set(ControlMode.PercentOutput, power);
        this._slave.set(ControlMode.PercentOutput, power);
        SmartDashboard.putNumber("Elev Count", this.getEncoder());
    }
    
    public double getEncoder()
    {
        return this._encoder.getDistance();
    }

    @Override
    protected void usePIDOutput(double output)
    {
        _master.set(ControlMode.PercentOutput, output);
    }

    @Override
    protected double returnPIDInput()
    {
        return _encoder.getDistance();
    }

    // Sets default command
    @Override
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        setDefaultCommand(new ElevatorByJoy());
    }
}