// //In the case that any of this goes wrong delete it all


// package frc.robot.commands.swervedrive;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.swervedrive.Turret;
// import frc.robot.subsystems.swervedrive.Vision;

// public class AimTurretCommand extends Command
// {
//     private final Vision vision;
//     private final Turret turret;

//     public AimTurretCommand(Vision vision, Turret turret)
//     {
//         this.vision = vision;
//         this.turret = turret;
//         addRequirements(turret);
//     }

//     @Override
//     public void execute()
//     {
//         System.out.println("Has Target: " + vision.hasTarget());
//         System.out.println("Target YAW: " + vision.getTargetYaw());
//         if (vision.hasTarget())
//         {
//             double yawError = vision.getTargetYaw();
//             turret.aimAtTarget(yawError);
//         }
//         else
//         {
//             turret.stopTurret();
//         }
//     }

//     @Override
//     public void end(boolean interrupted)
//     {
//         turret.stopTurret();
//     }

//     @Override
//     public boolean isFinished()
//     {
//         return false;
//     }
// }