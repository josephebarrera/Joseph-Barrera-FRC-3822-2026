package frc.robot.subsystems.swervedrive;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;

public class VisionSubsystem extends SubsystemBase
{

    private final PhotonCamera camera = new PhotonCamera("bodyCam");
    private final PhotonPoseEstimator poseEstimator;

    public VisionSubsystem()
    {
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Replace this later with your actual camera position on the robot.
        Transform3d robotToCamera = new Transform3d();

        poseEstimator = new PhotonPoseEstimator(fieldLayout,PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,robotToCamera);
    }

    public Optional<EstimatedRobotPose> getEstimatedPose()
    {

        var result = camera.getLatestResult();

        if(!result.hasTargets())
        {
            return Optional.empty();
        }

        return poseEstimator.update(result);
    }

    @Override
    public void periodic()
    {

        var result = camera.getLatestResult();

        System.out.println("Targets: " + result.hasTargets());

        if (result.hasTargets())
        {
            System.out.println("Tag ID: " + result.getBestTarget().getFiducialId());

            var estimate = poseEstimator.update(result);

            if (estimate.isPresent())
            {
                System.out.println("Pose: " + estimate.get().estimatedPose.toPose2d());
            } 
             else
            {
                System.out.println("Pose estimate failed");
            }
        }
    }

}
