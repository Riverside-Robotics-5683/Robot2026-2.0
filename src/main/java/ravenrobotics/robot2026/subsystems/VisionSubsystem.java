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
    private final PhotonCamera hopperCamera;

    private final PhotonPoseEstimator flywheelPoseEstimator;
    private final PhotonPoseEstimator hopperPoseEstimator;

    private List<PhotonPipelineResult> flywheelResults;
    private List<PhotonPipelineResult> hopperResults;

    private Matrix<N3, N1> flywheelCurrentStdDevs = VecBuilder.fill(0, 0, 0);
    private Matrix<N3, N1> hopperCurrentStdDevs = VecBuilder.fill(0, 0, 0);

    private final PoseEstimateConsumer poseConsumer;

    @FunctionalInterface
    public static interface PoseEstimateConsumer {
        public void accept(Pose2d pose, double poseTimestamp, Matrix<N3, N1> estimatedStdDevs);
    }

    public VisionSubsystem(PoseEstimateConsumer estimatedPoseConsumer) {
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        flywheelCamera = new PhotonCamera(VisionConstants.FLYWHEEL_CAMERA);
        hopperCamera = new PhotonCamera(VisionConstants.HOPPER_CAMERA);

        flywheelPoseEstimator = new PhotonPoseEstimator(fieldLayout, VisionConstants.FLYWHEEL_CAMERA_OFFSET);
        hopperPoseEstimator = new PhotonPoseEstimator(fieldLayout, VisionConstants.HOPPER_CAMERA_OFFSET);

        this.poseConsumer = estimatedPoseConsumer;
    }

    @Override
    public void periodic() {
        flywheelResults = flywheelCamera.getAllUnreadResults();

        Optional<EstimatedRobotPose> flywheelEstimatedPose = Optional.empty();

        for (var result: flywheelResults) {
            flywheelEstimatedPose = flywheelPoseEstimator.estimateCoprocMultiTagPose(result);

            if (flywheelEstimatedPose.isEmpty()) {
                flywheelEstimatedPose = flywheelPoseEstimator.estimateLowestAmbiguityPose(result);
            }

            flywheelCurrentStdDevs = computeEstimationStdDevs(flywheelEstimatedPose, result.getTargets());

            flywheelEstimatedPose.ifPresent(
                est -> {
                    var estStdDevs = getFlywheelStdDevs();

                    poseConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);

                    DogLog.log("Vision/FlywheelEstimatedPosition", est.estimatedPose.toPose2d());
                }
            );
        }

        hopperResults = hopperCamera.getAllUnreadResults();

        Optional<EstimatedRobotPose> hopperEstimatedPose = Optional.empty();

        for (var result: hopperResults) {
            hopperEstimatedPose = hopperPoseEstimator.estimateCoprocMultiTagPose(result);

            if (hopperEstimatedPose.isEmpty()) {
                hopperEstimatedPose = hopperPoseEstimator.estimateLowestAmbiguityPose(result);
            }

            hopperCurrentStdDevs = computeEstimationStdDevs(hopperEstimatedPose, result.getTargets());

            hopperEstimatedPose.ifPresent(
                est -> {
                    var estStdDevs = getHopperStdDevs();

                    poseConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);

                    DogLog.log("Vision/HopperEstimatedPosition", est.estimatedPose.toPose2d());
                }
            );
        }
    }

    public Matrix<N3, N1> getFlywheelStdDevs() {
        return flywheelCurrentStdDevs;
    }

    public Matrix<N3, N1> getHopperStdDevs() {
        return hopperCurrentStdDevs;
    }

    private Matrix<N3, N1> computeEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            return VisionConstants.singleTagDevs;
        }

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
            return VisionConstants.singleTagDevs;
        }

        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = VisionConstants.multiTagDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        return estStdDevs;
    }
}
