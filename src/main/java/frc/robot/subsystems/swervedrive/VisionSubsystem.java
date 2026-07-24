package frc.robot.subsystems.swervedrive;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase
{

    private final PhotonCamera turretCam = new PhotonCamera("turretCam");
    private final PhotonCamera bodyCam = new PhotonCamera("bodyCam");

    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private final PhotonPoseEstimator bodyPoseEstimator;

    private static final int[] HUB_TAG_IDS = {9, 10, 25, 26};

    public VisionSubsystem()
    {
        // bodyCam is mounted 0.3556m behind robot center, 0.127m right, 0.3556m up, facing straight backward
        // (yaw offset by PI since it faces the opposite direction from the robot's front), mounted level.
        Transform3d robotToCamera = new Transform3d(new Translation3d(-0.3556, -0.127, 0.3556), new Rotation3d(0.0, 0.0, Math.PI));

        bodyPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToCamera);
    }

    public PhotonPipelineResult getTurretResult()
    {
        return turretCam.getLatestResult();
    }

    public PhotonPipelineResult getBodyResult()
    {
        return bodyCam.getLatestResult();
    }

    /**
     * Full field pose estimate from bodyCam, for the drivetrain's odometry - fuse via
     * SwerveSubsystem.periodic() -> swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(...).
     */
    public Optional<EstimatedRobotPose> getEstimatedPose()
    {
        var result = bodyCam.getLatestResult();

        if (!result.hasTargets())
        {
            return Optional.empty();
        }

        return bodyPoseEstimator.update(result);
    }

    /**
     * The hub's fixed field position, derived from the known hub AprilTag poses in the loaded field layout
     * rather than a hardcoded/guessed coordinate. Averages whichever hub tags are present in the layout for
     * robustness. Returns empty if none of the hub tag IDs are found in the field layout.
     */
    public Optional<Pose2d> getHubFieldPosition()
    {
        double sumX = 0;
        double sumY = 0;
        int count = 0;

        for (int id : HUB_TAG_IDS)
        {
            Optional<edu.wpi.first.math.geometry.Pose3d> tagPose = fieldLayout.getTagPose(id);
            if (tagPose.isPresent())
            {
                sumX += tagPose.get().getX();
                sumY += tagPose.get().getY();
                count++;
            }
        }

        if (count == 0)
        {
            return Optional.empty();
        }

        return Optional.of(new Pose2d(sumX / count, sumY / count, new edu.wpi.first.math.geometry.Rotation2d()));
    }

    /**
     * Straight-line distance from the robot's current field position to the hub's fixed field position, in feet.
     * Doesn't require the turret camera to have a visual lock - it's always available as long as the robot's
     * own pose is known, which is what makes shooting from any position (not just when the hub happens to be
     * in view) possible.
     *
     * @param robotPose Current robot field pose, from SwerveSubsystem.getPose().
     */
    public Optional<Double> getPoseBasedDistanceToHubFeet(Pose2d robotPose)
    {
        return getHubFieldPosition().map(hubPose ->
            edu.wpi.first.math.util.Units.metersToFeet(hubPose.getTranslation().getDistance(robotPose.getTranslation())));
    }

    private boolean isHubTag(int id)
    {
        return id == 9 || id == 10 || id == 25 || id == 26;
    }

    public boolean hasTurretHubTarget()
    {
        var result = turretCam.getLatestResult();

        if (!result.hasTargets())
        {
            return false;
        }

        for (var target : result.getTargets())
        {
            if (isHubTag(target.getFiducialId()))
            {
                return true;
            }
        }

        return false;
    }

    public double getTurretHubYaw()
    {
        var result = turretCam.getLatestResult();

        if (!result.hasTargets())
        {
            return 0.0;
        }

        for (var target : result.getTargets())
        {
            if (isHubTag(target.getFiducialId()))
            {
                return target.getYaw();
            }
        }

        return 0.0;
    }

}
