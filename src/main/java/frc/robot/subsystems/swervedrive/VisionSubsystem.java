package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase
{
    
    private final PhotonCamera turretCam = new PhotonCamera("turretCam");
    private final PhotonCamera bodyCam = new PhotonCamera("bodyCam");

    public VisionSubsystem()
    {
    }

    public PhotonPipelineResult getTurretResult()
    {
        return turretCam.getLatestResult();
    }

    public PhotonPipelineResult getBodyResult()
    {
        return bodyCam.getLatestResult();
    }

   private boolean isHubTag(int id)
    {
        return (id == 9 || id == 10);
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

    public double getTurretHubPitch()
    {
        var result = turretCam.getLatestResult();
        if(!result.hasTargets())
        {
            return 0.0;
        }
        for (var target : result.getTargets())
        {
            if(isHubTag(target.getFiducialId()))
            {
                return target.getPitch();
            }
        }

        return 0.0;
    }

    public double getTurretHubDistanceFeet()
    {
        //CHANGE THIS LATER TO REAL NUMBERS
        double cameraHeightInches = 22.0;
        double targetHeightInches = 46.0;
        double cameraAngleDegrees = 28.0;

        double pitchDegrees = getTurretHubPitch();
        double totalAngleDegrees = cameraAngleDegrees + pitchDegrees;

        if(Math.abs(totalAngleDegrees) < 0.001)
        {
            return 0.0;
        }

        double distanceInches = (targetHeightInches - cameraHeightInches)
        / Math.tan(Math.toRadians(totalAngleDegrees));

        return distanceInches / 12.0;
    }

    // public boolean hasTurretTarget()
    // {
    //     return getTurretResult().hasTargets();
    // }

    // public double getTurretYaw()
    // {
    //     if (hasTurretTarget())
    //     {
    //         return getTurretResult().getBestTarget().getYaw();
    //     }
    //     return 0.0;
    // }

    // public double getTurretPitch()
    // {
    //     if (hasTurretTarget())
    //     {
    //         return getTurretResult().getBestTarget().getPitch();
    //     }
    //     return 0.0;
    // }


}