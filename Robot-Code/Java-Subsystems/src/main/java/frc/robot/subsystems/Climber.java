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
 * Climber class
 */
public class Climber extends Subsystem
{
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private DoubleSolenoid _solenoid1;  // The double solenoid
    private DoubleSolenoid _solenoid2;  // The double solenoid
    //private Compressor _compressor;  // Compressor (maybe not needed)
    private boolean _isExtended;  // True if the DoubleSolenoid is extended, false otherwise

    // Constructor
    public Climber()
    {
        this._solenoid1 = new DoubleSolenoid(RobotMap.Motors.Climber.FORWARD_SOLENOID1,
                                            RobotMap.Motors.Climber.REVERSE_SOLENOID1);
        this._solenoid2 = new DoubleSolenoid(RobotMap.Motors.Climber.FORWARD_SOLENOID2,
                                            RobotMap.Motors.Climber.REVERSE_SOLENOID2);
        //this._compressor = new Compressor(RobotMap.Motors.Climber.COMPRESSOR);
        //this._compressor.start();  // Maybe not needed
        this._isExtended = false;  // Default state is not extended
    }

    // Toggles the solenoid between extended and not extended
    public void toggleSolenoid()
    {
        this._isExtended = !this._isExtended;
        if (this._isExtended)
        {
            this._solenoid1.set(DoubleSolenoid.Value.kForward);
            this._solenoid2.set(DoubleSolenoid.Value.kForward);
        }
        else
        {
            this._solenoid1.set(DoubleSolenoid.Value.kReverse);
            this._solenoid2.set(DoubleSolenoid.Value.kForward);
        }
    }

    // Stops the solenoid
    public void stopSolenoid()
    {
        this._solenoid1.set(DoubleSolenoid.Value.kOff);
        this._solenoid2.set(DoubleSolenoid.Value.kForward);
    }
    
    @Override
    public void initDefaultCommand()
    {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new DriveByJoy());
        setDefaultCommand(new ClimberStop());
    }
}
