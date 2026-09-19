package frc.robot;

import static frc.robot.Constants.VisionConstants.EXAMPLE_TAG_ID;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Vision;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class VisionTest {

    private RobotContainer robotContainer;
    private Vision vision;

    @BeforeEach
    void setup() {
        if (!HAL.initialize(500, 0)) {
            throw new IllegalStateException("HAL failed to initialize");
        }
        robotContainer = new RobotContainer();
        vision = robotContainer.vision;
    }

    @AfterEach
    void teardown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        HAL.shutdown();
    }

    @Test
    void getTagPoseReturnsKnownTag() {
        assertTrue(vision.getTagPose(EXAMPLE_TAG_ID).isPresent());
    }

    @Test
    void getTagPoseReturnsEmptyForUnknownTag() {
        assertTrue(vision.getTagPose(9999).isEmpty());
    }

    @Test
    void noMeasurementWithoutCameraData() {
        CommandScheduler.getInstance().run();
        assertTrue(vision.getBestVisionMeasurementIfFresh().isEmpty());
    }

    @Test
    void visionNotAvailableWithoutCameraData() {
        CommandScheduler.getInstance().run();
        assertFalse(vision.isAnyVisionAvailable());
    }
}
