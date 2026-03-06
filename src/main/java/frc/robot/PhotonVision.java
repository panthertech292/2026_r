package frc.robot;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
//import org.photonvision.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Optional;

public class PhotonVision{
    PhotonCamera camera;
    PhotonCamera leftCamera;
    PhotonCamera rightCamera;
    AprilTagFieldLayout tagLayout;
    Transform3d leftRobotToCam;
    Transform3d rightRobotToCam;
    PhotonPoseEstimator leftEstimator;
    PhotonPoseEstimator rightEstimator;

    public PhotonVision(){
        // Initialize camera
        camera = new PhotonCamera("CameraName");
        leftCamera = new PhotonCamera("LEFT CAMERA NAME");
        rightCamera = new PhotonCamera("RIGHT CAMERA NAME");
        tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        leftRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d());
        rightRobotToCam = new Transform3d(new Translation3d(-0.5, 0.0, 0.5), new Rotation3d(0, Math.PI, 0));
        leftEstimator = new PhotonPoseEstimator(tagLayout, leftRobotToCam);
        rightEstimator = new PhotonPoseEstimator(tagLayout, rightRobotToCam);
    }
    public double[] getClosestBall(){
        // Inside your periodic loop
        var result = camera.getLatestResult();

        if (result.hasTargets()) {
            // Get the best target
            PhotonTrackedTarget target = result.getBestTarget();
            
            // Get Target Info
            double yaw = target.getYaw();         // Left/Right angle
            //double pitch = target.getPitch();     // Up/Down angle (Not needed, we don't care about vertical angle)
            double area = target.getArea();       // Target area %
            double skew = target.getSkew();       // Target rotation
            double[] closestBall = {yaw, area, skew};
            return closestBall;
        }
        return null;
    }

    public Pose2d getAprilTagPosition(){
        //AprilTags
        Pose2d bestLeftEstimate = null;
        int leftTagsSeen = -1;

        for (var leftresult : leftCamera.getAllUnreadResults()) {

            Optional<EstimatedRobotPose> visionEst = leftEstimator.estimateCoprocMultiTagPose(leftresult);

            if (visionEst.isPresent()) {
                int tags = leftresult.getTargets().size();
                if (tags > leftTagsSeen) {
                    bestLeftEstimate = visionEst.get().estimatedPose.toPose2d();
                    leftTagsSeen = tags;
                }
            }
        }

        Pose2d bestRightEstimate = null;
        int rightTagsSeen = -1;

        for (var rightresult : rightCamera.getAllUnreadResults()) {

            Optional<EstimatedRobotPose> visionEst = rightEstimator.estimateCoprocMultiTagPose(rightresult);

            if (visionEst.isPresent()) {
                int tags = rightresult.getTargets().size();
                if (tags > rightTagsSeen) {
                    bestRightEstimate = visionEst.get().estimatedPose.toPose2d();
                    rightTagsSeen = tags;
                }
            }
        }

        if (bestLeftEstimate != null || bestRightEstimate != null) {
            double fusedX = 0;
            double fusedY = 0;
            double fusedRotation = 0;
            int totalTags = leftTagsSeen + rightTagsSeen;

            if (totalTags == 0) totalTags = 1;

            if (bestLeftEstimate != null) {
                double weight = (double) leftTagsSeen / totalTags;
                fusedX += bestLeftEstimate.getX() * weight;
                fusedY += bestLeftEstimate.getY() * weight;
                fusedRotation += bestLeftEstimate.getRotation().getRadians() * weight;
            }

            if (bestRightEstimate != null) {
                double weight = (double) rightTagsSeen / totalTags;
                fusedX += bestRightEstimate.getX() * weight;
                fusedY += bestRightEstimate.getY() * weight;
                fusedRotation += bestRightEstimate.getRotation().getRadians() * weight;
            }

            Pose2d fusedPose = new Pose2d(fusedX, fusedY, new Rotation2d(fusedRotation));
            double visionX = fusedPose.getX();
            double visionY = fusedPose.getY();
            double visionRotation = fusedPose.getRotation().getDegrees();

            System.out.println("X: " + visionX);
            System.out.println("Y: " + visionY);
            System.out.println("Rotation: " + visionRotation);
            return fusedPose;
        } else {
            System.out.println("No tags detected by either camera.");
        }
        return null;
    }
}