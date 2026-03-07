// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class PhotonVision {
    PhotonCamera LeftCamera;
    PhotonCamera RightCamera;
    PhotonPoseEstimator LeftPoseEstimator;
    PhotonPoseEstimator RightPoseEstimator;
    AprilTagFieldLayout AMField;
    Translation3d LeftCameraPosition;
    Translation3d RightCameraPosition;
    Rotation3d LeftCameraRotation;
    Rotation3d RightCameraRotation;
    private Matrix<N3, N1> curStdDevsLeft;
    private Matrix<N3, N1> curStdDevsRight;
    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8); //TODO: Find these
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1); //TODO: Find these

    public PhotonVision(){
        LeftCamera = new PhotonCamera("OV9281-LEFT-APRIL");
        RightCamera = new PhotonCamera("OV9281-RIGHT-APRIL");
        AMField = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        LeftCameraPosition= new Translation3d(); //TODO: Update this
        RightCameraPosition = new Translation3d(); //TODO: Update this
        LeftCameraRotation = new Rotation3d(); //TODO: Update this
        RightCameraRotation = new Rotation3d(); //TODO: Update this

        LeftPoseEstimator = new PhotonPoseEstimator(AMField, new Transform3d(LeftCameraPosition, LeftCameraRotation));
        RightPoseEstimator = new PhotonPoseEstimator(AMField, new Transform3d(RightCameraPosition, RightCameraRotation));
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseLeftCamera(){
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : LeftCamera.getAllUnreadResults()) {
            visionEst = LeftPoseEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = LeftPoseEstimator.estimateLowestAmbiguityPose(result); //fallback if multi-tag gives nothing
            }
            updateEstimationStdDevsLeft(visionEst, result.getTargets());
        }
        return visionEst;
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseRightCamera(){
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : RightCamera.getAllUnreadResults()) {
            visionEst = RightPoseEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = RightPoseEstimator.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevsRight(visionEst, result.getTargets());
        }
        return visionEst;
    }
    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevsLeft(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevsLeft = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = LeftPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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
                curStdDevsLeft = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevsLeft = estStdDevs;
            }
        }
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevsRight(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevsRight = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = RightPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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
                curStdDevsRight = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevsRight = estStdDevs;
            }
        }
    }
    public Matrix<N3, N1> getEstimationStdDevsLeft() {
        return curStdDevsLeft;
    }
    public Matrix<N3, N1> getEstimationStdDevsRight() {
        return curStdDevsRight;
    }
}
