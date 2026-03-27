package frc.robot.subsystems.swervedrive;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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

    //PID for aiming
    private final PIDController aimPID = new PIDController(0.03, 0.0, 0.0);

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

    public void aimAtTarget(double yawErrorDegrees)
    {
        if(Math.abs(yawErrorDegrees) < 1.5)
        {
            stopTurret();
            return;
        }
        
        double output = aimPID.calculate(yawErrorDegrees, 0.0);

        if (Math.abs(output) < 0.08)
        {
            output = Math.copySign(0.08, output);
        }

        setTurretPower(output);
    }

    public boolean aimedAtTarget()
    {
        return aimPID.atSetpoint();
    }

    public double getTurretAngle()
    {
        return turretEncoder.getPosition();
    }

    public Command aimWithVision(frc.robot.subsystems.swervedrive.VisionSubsystem vision)
    {
        return Commands.run(() ->
        {
          
            if (vision.hasTurretHubTarget())
            {
                double yaw = vision.getTurretHubYaw();
                double pitch = vision.getTurretHubPitch();
                double distance = vision.getTurretHubDistanceFeet();

                aimAtTarget(-yaw);

                System.out.println("Hub Tag Found");
                System.out.println("Yaw" + yaw);
                System.out.println("Pitch: " + pitch);
                System.out.println("Distance:" + distance);
            }
            else
            {
                stopTurret();
            }
        }, this).finallyDo(() -> stopTurret());
    }

    @Override
    public void periodic()
    {
        System.out.println("Turret Position: " + turretEncoder.getPosition());
    } 

}