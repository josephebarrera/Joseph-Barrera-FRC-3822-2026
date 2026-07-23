package frc.robot.subsystems.swervedrive;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase
{
    //Motors
    SparkMax shooterLeft = new SparkMax(12, MotorType.kBrushless);
    SparkMax shooterRight = new SparkMax(13, MotorType.kBrushless);
    SparkMax shooterIntake = new SparkMax(14, MotorType.kBrushless);

    //Distance (feet) -> top shooter speed lookup, linearly interpolated between calibration points.
    //TODO - NOT TUNED YET: these points are only carried over from the old 3-tier distance check as a rough
    //starting shape (<4ft:0.7, <6ft:0.75, else:1.0) plus one placeholder far-range point. Replace with real
    //calibration points gathered by test-shooting at measured distances.
    private final InterpolatingDoubleTreeMap distanceToSpeed = new InterpolatingDoubleTreeMap();

    //How far out the top shooter should proactively spin up in anticipation of a shot - wider than the
    //realistic firing range so it has momentum built up by the time the driver is actually in position and
    //ready to fire. Outside this range the shooter is off, which is what avoids leaving it spinning and
    //draining the battery. TODO - NOT TUNED YET.
    private static final double AUTO_SPIN_RANGE_FEET = 12.0;

    public Shooter()
    {
        distanceToSpeed.put(4.0, 0.70);
        distanceToSpeed.put(6.0, 0.75);
        distanceToSpeed.put(10.0, 1.0);
    }

    /**
     * Placeholder hook for a future match-timing/scoring-window gate (e.g. only certain alliances can score
     * during certain phases). Always returns true for now since exact game timing rules aren't nailed down
     * yet - wire in a real check here once they are, without needing to touch autoSpinUp() itself.
     */
    private boolean isScoringAllowed()
    {
        return true;
    }

    /**
     * Continuously keeps the top shooter spun to the correct speed for the robot's current pose-based
     * distance to the hub, while within AUTO_SPIN_RANGE_FEET and scoring is allowed - off otherwise. No
     * button needed to start or stop this. Firing (feeding a ball via the agitator) is completely separate -
     * this only spins the flywheels, it never launches anything on its own. Meant to be set as the
     * shooter's default command.
     *
     * @param vision Shared VisionSubsystem instance.
     * @param drivebase Shared SwerveSubsystem instance, for the robot's current pose.
     */
    public Command autoSpinUp(VisionSubsystem vision, SwerveSubsystem drivebase)
    {
        return Commands.run(() ->
        {
            var distanceFeet = vision.getPoseBasedDistanceToHubFeet(drivebase.getPose());

            if (distanceFeet.isPresent() && distanceFeet.get() <= AUTO_SPIN_RANGE_FEET && isScoringAllowed())
            {
                setShooterSpeed(distanceToSpeed.get(distanceFeet.get()));
            }
            else
            {
                setShooterSpeed(0.0);
            }
        }, this);
    }

    //************************************************* Commands *************************************************/
    public Command spinShooterIntake()
    {
        return Commands.runOnce(()->
        {
            shooterIntake.set(-100.0);
        });
    }

    public Command stopShooterIntake()
    {
        return Commands.runOnce(()->
        {
            shooterIntake.set(0.0);
        });
    }
    
    public Command spinTopShooterReverse() 
    {
        return Commands.runOnce(()->
        {
            shooterLeft.set(100.0);
            shooterRight.set(-100.0);
        });
    }

    public Command shooterIntakeReverse()
    {
        return Commands.runOnce(()->
        {
            shooterIntake.set(100);
        });
       
    }

    public Command spinTopShooter(double speed) 
    {
        return Commands.run(() ->
        {
            shooterLeft.set(speed);
            shooterRight.set(-speed);
        })
        .finallyDo(() ->
        {
            shooterLeft.set(0.0);
            shooterRight.set(0.0);
        });
    }

    public Command spinTopShooter() 
    {
        return spinTopShooter(1.0);
    }

    //NEW
    public void setShooterSpeed(double speed)
    {
        shooterLeft.set(speed);
        shooterRight.set(-speed);
    }   

    public Command shootForward() 
    {
        return Commands.parallel(
            spinShooterIntake(),
            spinTopShooter()
        );
    }

    public Command shootStop() 
    {
        return Commands.parallel(
            stopShooterIntake(),
            stopTopShooter()
        );
    }

    public Command stopTopShooter() 
    {
        return Commands.runOnce(()->
        {
            shooterLeft.set(0.0);
            shooterRight.set(0.0);
        });
    }

    public Command goToLaunchAngle(double angle) 
    {
        return Commands.runOnce(()->{});
    }

    public Command turnToAngle(double angle) 
    {
        return Commands.runOnce(()->{});
    }

    //**************************************************** Auto Commands *****************************************************/
    public Command startTopShooterAuto()
    {
        return Commands.runOnce(()->
        {
            shooterLeft.set(0.65);
            shooterRight.set(-0.65);
        });
    }

    public Command startShooterIntakeAuto()
    {
        return Commands.runOnce(()->
        {
            shooterIntake.set(-100);
        });
    }
}