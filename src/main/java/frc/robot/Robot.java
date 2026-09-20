package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Extends {@link TimedRobot} directly and calls the scheduler explicitly from
 * {@code robotPeriodic()} -- Java's {@code commands2} library provides no
 * {@code TimedCommandRobot} convenience class that does this automatically, so this
 * override is required.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    public RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for
     * any initialization code.
     */
    @Override
    public void robotInit() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /** This autonomous runs the autonomous command selected by {@link RobotContainer}. */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousExit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopInit() {
        // This makes sure autonomous stops running when teleop starts. If you want
        // autonomous to continue until interrupted by another command, remove this.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }
}
