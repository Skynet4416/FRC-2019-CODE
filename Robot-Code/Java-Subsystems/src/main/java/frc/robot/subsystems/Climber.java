/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ClimberStop;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem
{
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private DoubleSolenoid _solenoid;
    private Compressor _compressor;
    private boolean _state;

    public Climber()
    {
        this._solenoid = new DoubleSolenoid(RobotMap.Motors.Climber.FORWARD_SOLENOID,
                                            RobotMap.Motors.Climber.REVERSE_SOLENOID);
        this._compressor = new Compressor(RobotMap.Motors.Climber.COMPRESSOR);
        this._compressor.start();
        this._state = false;
    }

    public void toggleSolenoid()
    {
        this._state = !this._state;
        if (this._state)
        {
            this._solenoid.set(DoubleSolenoid.Value.kForward);
        }
        else
        {
            this._solenoid.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public void stopSolenoid()
    {
        this._solenoid.set(DoubleSolenoid.Value.kOff);
    }
    
    @Override
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new DriveByJoy());
        setDefaultCommand(new ClimberStop());
    }
}
