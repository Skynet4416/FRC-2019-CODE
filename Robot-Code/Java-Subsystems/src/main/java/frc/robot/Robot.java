/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.*;

import java.util.HashMap;

import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
    // subsystem declarations
    public static OI oi;
    public static Roller roller;
    public static Chassis chassis;
    public static Claw claw;
    public static Climber climber;

    private static void displayDashboardNum(String key, double defaultValue)
    {
        SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
    }

    public static final HashMap<String, Double> dashboardEntries = new HashMap<String, Double>()
    {
        private static final long serialVersionUID = 1L;
        {
            put("elev_kp", 0.0);
            put("elev_ki", 0.0);
            put("elev_kd", 0.0);
            put("claw_kp", 0.0);
            put("claw_ki", 0.0);
            put("claw_kd", 0.0);
            put("Static", 0.0);
        }
    }; 
    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    
    @Override
    public void robotInit()
    {
        //init smartdashboard values
        SmartDashboard.putString("Reached", "0");
        for (String key : dashboardEntries.keySet())
        {
            displayDashboardNum(key, dashboardEntries.get(key));
        }
        SmartDashboard.putBoolean("Initiated", false);
        roller = new Roller();
        chassis = new Chassis();
        claw = new Claw();
        climber = new Climber();
        CameraServer.getInstance().startAutomaticCapture(0);//.setResolution(352, 240);
        CameraServer.getInstance().startAutomaticCapture(1);//.setResolution(352, 240);
        oi = new OI();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic()
    {

    }

    /**
     * This function is called once each time the robot enters Disabled mode. You
     * can use it to reset any subsystem information you want to clear when the
     * robot is disabled.
     */
    @Override
    public void disabledInit()
    {

    }

    @Override
    public void disabledPeriodic()
    {
        Scheduler.getInstance().run();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString code to get the
     * auto name from the text box below the Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional commands to the
     * chooser code above (like the commented example) or additional comparisons to
     * the switch structure below with additional strings & commands.
     */
    @Override
    public void autonomousInit()
    {

    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic()
    {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit()
    {

    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic()
    {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic()
    {
    }
}
