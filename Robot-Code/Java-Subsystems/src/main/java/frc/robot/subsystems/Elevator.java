
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
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
    public double STATIC_VOLTAGE = 0.3;
    private Encoder _encoder = new Encoder(RobotMap.PIDConstants.Elevator.A_CHANNEL, RobotMap.PIDConstants.Elevator.B_CHANNEL, false, EncodingType.k4X);
	public static final double TOLERANCE = 15;
	public static final double DISTANCE_PER_TICK = 0.007624 * Math.PI / 512;

    public Elevator()
    {
        // super(system name, proportional, integral, differential)
      	super("Elevator", SmartDashboard.getNumber("elev_kp", 0), SmartDashboard.getNumber("elev_ki", 0),
            SmartDashboard.getNumber("elev_kd", 0));
        _slave.set(ControlMode.Follower, _master.getBaseID());
        _encoder.setDistancePerPulse(DISTANCE_PER_TICK);
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
        //Positive power goes up and negative goes down (Wanted outcome)
        this._master.set(ControlMode.PercentOutput, power);
        SmartDashboard.putNumber("Elev Count", this.getEncoder());
    }
    
    public double getEncoder()
    {
        return _encoder.getDistance();
    }

    @Override
    protected void usePIDOutput(double output)
    {
        
    }

    @Override
    protected double returnPIDInput()
    {
        return 0;
    }

    // Sets default command
    @Override
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        setDefaultCommand(new ElevatorByJoy());
    }
}