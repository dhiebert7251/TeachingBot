package frc.robot.commands;

import static frc.robot.Constants.METERS_PER_FOOT;
import static frc.robot.Constants.VisionConstants.EXAMPLE_TAG_ID;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

// Lives in frc.robot.commands (not frc.robot.subsystems) so it can reach
// ApproachTagCommand's own package-private phase/targetPoint/turnPid/drivePid fields;
// that's why it uses DriveTrain.setEncoderPositionsForTest() below instead of poking
// leftEncoder/rightEncoder directly -- package-private access can't span both packages.
class ApproachTagCommandTest {

    private RobotContainer robotContainer;
    private DriveTrain drivetrain;
    private Vision vision;

    @BeforeEach
    void setup() {
        if (!HAL.initialize(500, 0)) {
            throw new IllegalStateException("HAL failed to initialize");
        }
        robotContainer = new RobotContainer();
        drivetrain = robotContainer.drivetrain;
        vision = robotContainer.vision;
    }

    @AfterEach
    void teardown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        HAL.shutdown();
    }

    private void enable() {
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.notifyNewData();
    }

    private void step(double seconds) {
        SimHooks.stepTiming(seconds);
        CommandScheduler.getInstance().run();
    }

    @Test
    void failsForUnknownTag() {
        enable();
        step(0.02);

        ApproachTagCommand command = new ApproachTagCommand(drivetrain, vision, 9999, 3.0);
        command.schedule();
        step(0.02);
        assertFalse(command.isScheduled());
    }

    @Test
    void computesStandoffPoint() {
        Pose2d tagPose = vision.getTagPose(EXAMPLE_TAG_ID).get().toPose2d();

        ApproachTagCommand command = new ApproachTagCommand(drivetrain, vision, EXAMPLE_TAG_ID, 3.0);
        command.initialize();

        assertEquals(ApproachTagCommand.Phase.TURN_TO_TARGET, command.phase);
        double distanceFromTag = command.targetPoint.getDistance(tagPose.getTranslation());
        assertEquals(3.0 * METERS_PER_FOOT, distanceFromTag, 0.01);
    }

    @Test
    void progressesThroughAllPhases() {
        enable();
        step(0.02);

        SimDeviceSim navxSim = new SimDeviceSim("navX-Sensor[4]");
        var yawSim = navxSim.getDouble("Yaw");

        ApproachTagCommand command = new ApproachTagCommand(drivetrain, vision, EXAMPLE_TAG_ID, 3.0);
        command.schedule();
        step(0.02);
        assertTrue(command.isScheduled());
        assertEquals(ApproachTagCommand.Phase.TURN_TO_TARGET, command.phase);

        // getHeadingDegrees() negates the raw navX yaw (see DriveTrain), so the sign
        // is flipped here to match.
        double bearingDegrees = command.turnPid.getSetpoint();
        yawSim.set(-bearingDegrees);
        step(0.02);
        assertEquals(ApproachTagCommand.Phase.DRIVE_TO_TARGET, command.phase);

        double targetDistance = command.drivePid.getSetpoint();
        drivetrain.setEncoderPositionsForTest(targetDistance, targetDistance);
        step(0.02);
        assertEquals(ApproachTagCommand.Phase.FACE_TAG, command.phase);

        double finalHeadingDegrees = command.turnPid.getSetpoint();
        yawSim.set(-finalHeadingDegrees);
        step(0.02);
        assertFalse(command.isScheduled());
    }
}
