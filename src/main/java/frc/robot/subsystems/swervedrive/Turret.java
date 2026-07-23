package frc.robot.subsystems.swervedrive;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;

public class Turret extends SubsystemBase
{
    //Motor
    SparkMax turret = new SparkMax(11, MotorType.kBrushless);

    //Encoder
    private final RelativeEncoder turretEncoder = turret.getEncoder();

    //Limits
    private static final double MIN_TURRET_POSITION = -19;
    private static final double MAX_TURRET_POSITION = 0.0;

    //PID for aiming - gains are a starting point, to be tuned together on the real robot
    private final PIDController aimPID = new PIDController(0.03, 0.0, 0.0);

    //Feedforward to overcome static friction, applied in the direction of motion. Replaces the old crude
    //"clamp minimum output to 0.08" hack with the standard PID + feedforward structure - same starting
    //value, but now tunable independently of kP instead of being tangled into the output clamp.
    //TODO: measure the real minimum power needed to just barely start turret motion and refine this.
    private static final double kS = 0.08;

    //TODO - NOT MEASURED YET: the turret's degrees-per-encoder-unit relationship. Determine by rotating the
    //turret through its full physical range and correlating encoder values (MIN_TURRET_POSITION to
    //MAX_TURRET_POSITION) to measured real-world degrees. Pose-based aiming (aimAtFieldPosition) will aim
    //at the wrong angle until this is measured and corrected - vision-based aiming (aimAtTarget driven
    //directly by camera yaw) is unaffected since it doesn't use this conversion.
    private static final double DEGREES_PER_ENCODER_UNIT = 1.0;

     public Turret()
    {
        aimPID.setTolerance(1.0);
    }

    public void setTurretPower(double power)
    {
        double position = getTurretAngle();

        //STOP if trying to go past left limit
        if (position <= MIN_TURRET_POSITION && power < 0)
        {
            turret.set(0.0);
            return;
        }

        //STOP if trying to go past right limit
        if (position >= MAX_TURRET_POSITION && power > 0)
        {
            turret.set(0.0);
            return;
        }

        turret.set(MathUtil.clamp(power, -0.35, 0.35));
    }

    public void stopTurret()
    {
        turret.set(0.0);
    }

    public void testTurnLeft()
    {
        setTurretPower(-0.15);
    }

    public void testTurnRight()
    {
        setTurretPower(0.15);
    }

    /**
     * Drives the turret to null out a yaw/angle error in degrees, whatever the source (direct vision yaw error,
     * or a pose-calculated error from {@link #aimAtFieldPosition}). Always routes through setTurretPower(), so
     * the physical limit checks there apply regardless of which targeting method is driving it.
     */
    public void aimAtTarget(double yawErrorDegrees)
    {
        if(Math.abs(yawErrorDegrees) < 1.5)
        {
            stopTurret();
            return;
        }

        double output = aimPID.calculate(yawErrorDegrees, 0.0);
        output += Math.copySign(kS, output);

        setTurretPower(output);
    }

    /**
     * Aims the turret at the hub using the robot's known field position and the hub's fixed field position,
     * rather than requiring the turret camera to currently see the hub tag. Used as a fallback when the
     * camera doesn't have a lock (narrow field of view, tag briefly out of sight) but the hub may still be
     * within the turret's physical reach. Requires DEGREES_PER_ENCODER_UNIT to be measured for accuracy.
     *
     * @param robotPose Current robot field pose, from SwerveSubsystem.getPose().
     * @param hubFieldPosition The hub's fixed field position, from VisionSubsystem.getHubFieldPosition().
     */
    public void aimAtFieldPosition(Pose2d robotPose, Translation2d hubFieldPosition)
    {
        Translation2d robotToHub = hubFieldPosition.minus(robotPose.getTranslation());
        Rotation2d absoluteBearing = robotToHub.getAngle();
        Rotation2d desiredRobotRelativeAngle = absoluteBearing.minus(robotPose.getRotation());

        double desiredTurretDegrees = desiredRobotRelativeAngle.getDegrees();
        double currentTurretDegrees = getTurretAngle() * DEGREES_PER_ENCODER_UNIT;
        double errorDegrees = desiredTurretDegrees - currentTurretDegrees;

        //Normalize to -180..180 so we always compute the shorter turn direction. The turret's limited
        //physical range means a large swing will just cleanly hit a limit and stop, via setTurretPower.
        while (errorDegrees > 180)
        {
            errorDegrees -= 360;
        }
        while (errorDegrees < -180)
        {
            errorDegrees += 360;
        }

        aimAtTarget(errorDegrees);
    }

    public boolean aimedAtTarget()
    {
        return aimPID.atSetpoint();
    }

    public double getTurretAngle()
    {
        return turretEncoder.getPosition();
    }

    /**
     * Continuously tracks the hub: uses the turret camera's direct sighting when available (most accurate),
     * falling back to the robot's known field position (aimAtFieldPosition) when the camera doesn't have a
     * lock. Always routes through setTurretPower(), so the existing physical limit checks apply either way.
     * Meant to be set as the turret's default command so it tracks continuously without holding a button.
     *
     * @param vision Shared VisionSubsystem instance.
     * @param drivebase Shared SwerveSubsystem instance, for the robot's current pose.
     */
    public Command trackHub(VisionSubsystem vision, SwerveSubsystem drivebase)
    {
        return Commands.run(() ->
        {
            if (vision.hasTurretHubTarget())
            {
                aimAtTarget(-vision.getTurretHubYaw());
            }
            else
            {
                vision.getHubFieldPosition().ifPresentOrElse(
                    hubPosition -> aimAtFieldPosition(drivebase.getPose(), hubPosition.getTranslation()),
                    this::stopTurret
                );
            }
        }, this).finallyDo(() -> stopTurret());
    }

    @Override
    public void periodic()
    {
        //System.out.println("Turret Position: " + turretEncoder.getPosition());
    }

}
