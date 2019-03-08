/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.ClawPid;

/**
 * Claw subsystem
 */
public class Claw extends PIDSubsystem
{
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private TalonSRX _clawMotor = new TalonSRX(RobotMap.Motors.Claw.MOTOR);
	private Encoder _encoder = new Encoder(RobotMap.Sensors.Claw.A_CHANNEL, RobotMap.Sensors.Claw.B_CHANNEL, true, EncodingType.k4X);
    private DigitalInput _closeSwitch = new DigitalInput(RobotMap.Sensors.Claw.SWITCH);
    public static final double TOLERANCE = 0; // no tolerance means active resistance to game objects
    public static final double DEGREES_PER_TICK = 360.0 / 4096;
    public enum State {panel, cargo, closed, none};
    public static boolean initiated = false;
    public static final double PANEL_VAL = -1.93359375; //11.4;
    public static final double CARGO_VAL = -6.71; //6.9;
    public static final double CLOSE_VAL = 0; //13.4;
    public static final double KP = 0.3;
    public static final double KI = 0.003;
    public static final double KD = 0.2;
    /**
     * Powers the claw with supplied input
     * 
     * @param power percentage of power to supply to the motors. If invalid,
     *              rounded.
     */

    public Claw()
    {
		// super(system name, proportional, integral, differential)
      	super("Claw", KP, KI, KD);
        _clawMotor.setInverted(true);
		// makes range not overflow the motors
		setOutputRange(-1, 1);
        setAbsoluteTolerance(TOLERANCE);

        //configures encoder
        this._encoder.setDistancePerPulse(DEGREES_PER_TICK);
        SmartDashboard.putNumber("ClawPID", 0);
	}
    
    public boolean getSwitch()
    {
        return this._closeSwitch.get();
    }

    public void setZero()
    {
        SmartDashboard.putBoolean("Initiated", true);
        this._encoder.reset();
        Claw.initiated = true;
    }

    public void setPoint(double point)
    {
        if (Claw.initiated)
        {
            enable();
            setSetpoint(point);
        }
    }
    public void set(Claw.State state)
    {
        SmartDashboard.putNumber("ClawPID", this.getSetpoint());
        switch (state)
        {
            case panel:
                this.setPoint(PANEL_VAL);
                break;
            case closed:
                this.setPoint(CLOSE_VAL);
                break;
            case cargo:
                this.setPoint(CARGO_VAL);
                break;
        }
    }
    
    public void set(double power)
    {
        disable();
        _clawMotor.set(ControlMode.PercentOutput, (power / Math.abs(power)) * 0.2);
    }
	
    public double getEncoder()
    {
        return this._encoder.getDistance();
    }

    @Override
	protected double returnPIDInput()
	{
        return this.getEncoder();
    }

    @Override
	protected void usePIDOutput(double output)
	{
        //set(output);
        this._clawMotor.set(ControlMode.PercentOutput, output);
    }

    // Sets default command
    @Override
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        setDefaultCommand(new ClawPid());
    }

    public void reset()
    {
        _encoder.reset();
    }
}
