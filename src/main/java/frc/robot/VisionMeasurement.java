package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record VisionMeasurement(
    Pose2d estimatedPose,
    double timestampSeconds,
    Matrix<N3, N1> standardDeviations,
    int numTagsUsed
) {}
