/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.ClawStop;

/**
 * Claw subsystem
 */
public class Claw extends PIDSubsystem
{
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private TalonSRX _clawMotor = new TalonSRX(RobotMap.Motors.Claw.MOTOR);
	private Encoder _encoder = new Encoder(RobotMap.PIDConstants.Claw.A_CHANNEL, RobotMap.PIDConstants.Claw.B_CHANNEL, true, EncodingType.k4X);
	public static final double TOLERANCE = 15;
	public static final double DEGREES_PER_TICK = 360.0 / 1024;
    /**
     * Powers the claw with supplied input
     * 
     * @param power percentage of power to supply to the motors. If invalid,
     *              rounded.
     */

    public Claw()
    {
		// super(system name, proportional, integral, differential)
      	super("Claw", SmartDashboard.getNumber("claw_kp", 0), SmartDashboard.getNumber("claw_ki", 0),
			SmartDashboard.getNumber("claw_kd", 0));
		// makes range not overflow the motors
		setOutputRange(-1, 1);
		setAbsoluteTolerance(TOLERANCE);

		//configures encoder
		_encoder.setDistancePerPulse(DEGREES_PER_TICK);
	}
	
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
	
    public double getEncoder()
    {
        return _encoder.getDistance();
    }

    @Override
	protected double returnPIDInput()
	{
        return 0;
    }

    @Override
	protected void usePIDOutput(double output)
	{
        
    }

    // Sets default command
    @Override
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        setDefaultCommand(new ClawStop());
    }
}
