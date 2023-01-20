/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*This program modified by CWS 8/4/2021,   This will track the motion of an analog
 sensor which rotates beyond 360 deg.
 Note the pot is connected directly to the Robo Rio analog in
    The Talon sensor collection can measure the value without roll up, but it is too slow at 150ms.
    The talon PID cannot accommodate the 0/360 discontinuity
    Spark controls the drive motor (Neo) in speed mode.
 */

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

public class Robot extends TimedRobot {
    public final SendableChooser<Double> speedChooser = new SendableChooser<>();
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        speedChooser.setDefaultOption("Normal (Fast 100%)", 1.0);
        speedChooser.addOption("Mild (Medium 50%)", 0.5);
        speedChooser.addOption("Safe (Slow 25%)", 0.25);
        SmartDashboard.putData(speedChooser);
        SmartDashboard.putNumber("AngleOffset1", kDefaultPeriod);
        SmartDashboard.putNumber("AngleOffset2", kDefaultPeriod);
        SmartDashboard.putNumber("AngleOffset3", kDefaultPeriod);
        SmartDashboard.putNumber("AngleOffset4", kDefaultPeriod);

        robotContainer = new RobotContainer(this);
    }

    private String autoName = "";
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.updateSmartDashboard();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }
}