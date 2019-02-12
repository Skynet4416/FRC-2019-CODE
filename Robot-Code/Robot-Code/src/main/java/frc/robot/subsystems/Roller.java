/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.RollerStop;

/**
 * Add your docs here.
 */
public class Roller extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private SpeedController _rightMotor = new Talon(RobotMap.Motors.Roller.RIGHT);
  private SpeedController _leftMotor = new Talon(RobotMap.Motors.Roller.LEFT);
  /**
   * Powers the gripper with supplied input
   * @param power percentage of power to supply to the motors. If invalid, rounded.
   */
  public void set(double power)
  {
    if (power > 1)
    {// rounds down power over the maximum
      power = 1;
      System.out.println("Roller: rounded down power over 1");
    }
    if (power < -1)
    {// rounds up power under the minimum
      power = -1;
      System.out.println("Roller: rounded up power under -1");
    }
    // one of the motors needs to rotate opposite the other to make the gripper work
    _rightMotor.set(power);
    _leftMotor.set(-power);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new RollerStop());
  }
}
