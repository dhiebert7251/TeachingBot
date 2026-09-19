package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VisionMeasurement;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * DIVERGENCE FROM THE REAL COMPETITION ROBOT, disclosed deliberately: that code builds
 * {@code PhotonPoseEstimator} with an explicit {@code PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR}
 * and calls {@code poseEstimator.update(result)}. This file instead uses the 2-arg
 * constructor and calls {@code estimateCoprocMultiTagPose()}/{@code estimateLowestAmbiguityPose()}
 * directly -- the pattern verified against the Python sibling's installed photonlibpy,
 * NOT verified against this project's own Java PhotonLib jar. See README Verification
 * status before trusting this in a build.
 */
public class Vision extends SubsystemBase {

    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    private final PhotonCamera camera = new PhotonCamera(CAMERA_NAME);
    private final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(fieldLayout, ROBOT_TO_CAMERA);

    private double lastResultTimestamp = -1.0;
    private double lastFreshFrameFpgaTimestamp = -1.0;

    private Optional<VisionMeasurement> cachedMeasurement = Optional.empty();

    private int telemetryLoopCounter = 0;

    public Optional<Pose3d> getTagPose(int tagId) {
        return fieldLayout.getTagPose(tagId);
    }

    public Optional<VisionMeasurement> getBestVisionMeasurementIfFresh() {
        return cachedMeasurement.filter(this::isMeasurementFresh);
    }

    private boolean isMeasurementFresh(VisionMeasurement measurement) {
        return (Timer.getFPGATimestamp() - measurement.timestampSeconds()) <= MAX_VISION_AGE_SECONDS;
    }

    public boolean isAnyVisionAvailable() {
        return lastFreshFrameFpgaTimestamp >= 0
            && (Timer.getFPGATimestamp() - lastFreshFrameFpgaTimestamp) <= 0.5;
    }

    private Optional<VisionMeasurement> computeMeasurement(PhotonPipelineResult result) {
        if (!result.hasTargets()) {
            return Optional.empty();
        }

        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);
        if (estimatedPose.isEmpty()) {
            estimatedPose = poseEstimator.estimateLowestAmbiguityPose(result);
        }
        if (estimatedPose.isEmpty()) {
            return Optional.empty();
        }

        EstimatedRobotPose pose = estimatedPose.get();
        Pose2d pose2d = pose.estimatedPose.toPose2d();

        double fieldLength = fieldLayout.getFieldLength();
        double fieldWidth = fieldLayout.getFieldWidth();
        if (pose2d.getX() < 0 || pose2d.getX() > fieldLength || pose2d.getY() < 0 || pose2d.getY() > fieldWidth) {
            return Optional.empty(); // a pose off the field is never real
        }

        List<PhotonTrackedTarget> targets = result.getTargets();
        if (targets.size() == 1 && result.getBestTarget().getPoseAmbiguity() > MAX_AMBIGUITY) {
            return Optional.empty();
        }

        int numTags = pose.targetsUsed.size();
        Matrix<N3, N1> stdDevs;
        if (numTags >= MIN_TAGS_FOR_MULTI_TAG) {
            stdDevs = MULTI_TAG_STDDEVS;
        } else {
            double averageDistance = averageTagDistance(targets, pose2d);
            if (averageDistance > MAX_TAG_DISTANCE_METERS) {
                return Optional.empty();
            }
            stdDevs = averageDistance < 2.0 ? SINGLE_TAG_CLOSE_STDDEVS : SINGLE_TAG_FAR_STDDEVS;
        }

        return Optional.of(new VisionMeasurement(pose2d, pose.timestampSeconds, stdDevs, numTags));
    }

    private double averageTagDistance(List<PhotonTrackedTarget> targets, Pose2d robotPose) {
        double totalDistance = 0.0;
        int validTagCount = 0;
        for (PhotonTrackedTarget target : targets) {
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isPresent()) {
                totalDistance += robotPose.getTranslation().getDistance(tagPose.get().toPose2d().getTranslation());
                validTagCount++;
            }
        }
        return validTagCount == 0 ? Double.POSITIVE_INFINITY : totalDistance / validTagCount;
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();
        double timestamp = result.getTimestampSeconds();

        // A never-connected camera reports a placeholder timestamp near zero (not the
        // -1.0 this class starts at) -- `timestamp > 0` avoids treating that as fresh.
        if (timestamp > 0 && timestamp > lastResultTimestamp) {
            lastResultTimestamp = timestamp;
            lastFreshFrameFpgaTimestamp = Timer.getFPGATimestamp();
            cachedMeasurement = computeMeasurement(result);
        }

        telemetryLoopCounter++;
        if (telemetryLoopCounter >= TELEMETRY_PERIOD_LOOPS) {
            telemetryLoopCounter = 0;
            SmartDashboard.putBoolean("Vision/Available", isAnyVisionAvailable());
            SmartDashboard.putBoolean("Vision/HasMeasurement", cachedMeasurement.isPresent());
            cachedMeasurement.ifPresent(
                measurement -> SmartDashboard.putNumber("Vision/NumTagsUsed", measurement.numTagsUsed())
            );
        }
    }
}
