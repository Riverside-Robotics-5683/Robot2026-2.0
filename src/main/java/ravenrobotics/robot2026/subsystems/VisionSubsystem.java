package ravenrobotics.robot2026.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot2026.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

    private final AprilTagFieldLayout fieldLayout;

    private final PhotonCamera flywheelCamera;

    private final PhotonPoseEstimator flywheelPoseEstimator;

    private List<PhotonPipelineResult> flywheelResults;
    private Matrix<N3, N1> currentStdDevs;

    private final PoseEstimateConsumer poseConsumer;

    @FunctionalInterface
    public static interface PoseEstimateConsumer {
        public void accept(Pose2d pose, double poseTimestamp, Matrix<N3, N1> estimatedStdDevs);
    }

    public VisionSubsystem(PoseEstimateConsumer estimatedPoseConsumer) {
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        flywheelCamera = new PhotonCamera(VisionConstants.FLYWHEEL_CAMERA);

        flywheelPoseEstimator = new PhotonPoseEstimator(fieldLayout, VisionConstants.FLYWHEEL_CAMERA_OFFSET);

        this.poseConsumer = estimatedPoseConsumer;

        this.register();
    }

    @Override
    public void periodic() {
        flywheelResults = flywheelCamera.getAllUnreadResults();

        Optional<EstimatedRobotPose> estimatedPose = Optional.empty();

        for (var result: flywheelResults) {
            estimatedPose = flywheelPoseEstimator.estimateCoprocMultiTagPose(result);

            if (estimatedPose.isEmpty()) {
                estimatedPose = flywheelPoseEstimator.estimateLowestAmbiguityPose(result);
            }

            updateEstimationStdDevs(estimatedPose, result.getTargets());

            estimatedPose.ifPresent(
                est -> {
                    var estStdDevs = getStdDevs();

                    poseConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);

                    DogLog.log("Vision/FlywheelEstimatedPosition", est.estimatedPose.toPose2d());
                }
            );
        }
    }

    public Matrix<N3, N1> getStdDevs() {
        return currentStdDevs;
    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            currentStdDevs = VisionConstants.singleTagDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.singleTagDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = flywheelPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                currentStdDevs = VisionConstants.singleTagDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.multiTagDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                currentStdDevs = estStdDevs;
            }
        }
    }
}
