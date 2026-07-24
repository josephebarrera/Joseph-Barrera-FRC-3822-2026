package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swervedrive.Actuator;
import frc.robot.subsystems.swervedrive.Agitator;
import frc.robot.subsystems.swervedrive.Intake;
import frc.robot.subsystems.swervedrive.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Turret;
import frc.robot.subsystems.swervedrive.VisionSubsystem;

import java.io.File;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

    //Create Controllers
    final CommandXboxController driverXbox = new CommandXboxController(0);
    final CommandXboxController shooterXbox = new CommandXboxController(1);

    //The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

    //Create a vision subsystem
    private final VisionSubsystem vision = new VisionSubsystem();

    //Created a shooter
    Shooter shooter = new Shooter();

    //Created a agitator
    Agitator agitator = new Agitator();

    //Create a intake
    Intake intake = new Intake();

    //Create a turret
    Turret turret = new Turret();

    //Create a actuator
    Actuator actuator = new Actuator();

    //Give SmartDashboard the ability to choose Autos
    private final SendableChooser<Command> autos = new SendableChooser<>();

    /**
    * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
    */
    public SwerveInputStream driveInputStream = SwerveInputStream.of(
    drivebase.getSwerveDrive(),
    () -> driverXbox.getLeftY(),
    () -> driverXbox.getLeftX())
    .withControllerRotationAxis(()-> -driverXbox.getRightX())
    .deadband(Constants.OperatorConstants.DEADBAND)
    .scaleTranslation(0.5) //Originally 0.5 
    .allianceRelativeControl(false);
      
    /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
    public RobotContainer()
    {

      //Share the single VisionSubsystem instance with the drivetrain so bodyCam pose estimates feed odometry
      drivebase.setVisionSubsystem(vision);

      //Configure the PathPlanner commands
      setupPathPlannerCommands();

      //Configure the trigger bindings
      configureBindings();

      //Configure the SmartDashboard
      autos.setDefaultOption("Middle Auto", new PathPlannerAuto("Middle Auto"));

      //Display the options 
      SmartDashboard.putData("Auto Chooser", autos);

      //Silences Joystick warnings
      DriverStation.silenceJoystickConnectionWarning(true);

    }

    private void setupPathPlannerCommands()
    {
      NamedCommands.registerCommand("MARKER Top Shooter", Commands.runOnce(() -> System.out.println("MARKER FIRED: Start Top Shooter")));
      NamedCommands.registerCommand("MARKER Shoot Forward", Commands.runOnce(() -> System.out.println("MARKER FIRED: Shoot Forward")));
      NamedCommands.registerCommand("Start Top Shooter", shooter.startTopShooterAuto());
      NamedCommands.registerCommand("Shoot Forward", Commands.parallel(agitator.funnelForwardAuto(), shooter.startShooterIntakeAuto()));
    }

    private void configureBindings()
    {

      /*********************************************************** Driver Commands ***************************************************/
      //Single-controller setup: turret aim and top shooter speed are both automatic default commands now,
      //so one driver can handle driving, firing, and the manual turret/agitator fallbacks below.

      //Zero the gyro
      driverXbox.b()
        .onTrue(Commands.runOnce(drivebase::zeroGyroAllianceAware));

      Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveInputStream);

      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

      //Fire: hold right trigger to feed a ball from the agitator into the (already auto-spinning) shooter
      driverXbox.rightTrigger()
        .whileTrue(Commands.parallel(agitator.funnelForward(), shooter.spinShooterIntake()))
        .onFalse(Commands.parallel(agitator.funnelStop(), shooter.stopShooterIntake()));

      //Turret: manual nudge left/right - fallback override, interrupts the auto-tracking default command
      //while held, hands back to it automatically on release
      driverXbox.povLeft()
        .whileTrue(Commands.run(() -> turret.testTurnLeft(), turret))
        .onFalse(Commands.runOnce(() -> turret.stopTurret(), turret));

      driverXbox.povRight()
        .whileTrue(Commands.run(() -> turret.testTurnRight(), turret))
        .onFalse(Commands.runOnce(() -> turret.stopTurret(), turret));

      //Agitator and shooter intake reverse (e.g. to clear a jam)
      driverXbox.x()
        .whileTrue(Commands.parallel(agitator.funnelReverse(), shooter.shooterIntakeReverse()))
        .onFalse(Commands.parallel(agitator.funnelStop(), shooter.stopShooterIntake()));
      /*******************************************************************************************************************************/

      /****************************************************** Automatic default commands *********************************************/

      //Top shooter speed - auto spin-up/down by pose-based distance to the hub, no button needed
      shooter.setDefaultCommand(shooter.autoSpinUp(vision, drivebase));

      //Turret tracking - TEMPORARILY DISABLED while drivetrain testing is in progress (DEGREES_PER_ENCODER_UNIT
      //isn't measured yet and PID isn't tuned, so it was moving unpredictably). Re-enable this line once the
      //drivetrain is confirmed solid and it's time to work on turret calibration/tuning.
      // turret.setDefaultCommand(turret.trackHub(vision, drivebase));
      /*******************************************************************************************************************************/

      /****************************************************** Intake (out of scope - hardware currently broken) *********************/
      //Left on shooterXbox for now, unchanged, until the intake hardware is fixed and this gets revisited.

      //Intake: Toggle On and Off
      shooterXbox.b()
        .toggleOnTrue(intake.spinIntakeForward());

      //Open intake
      shooterXbox.povDown()
        .whileTrue((intake.foldOpenIntake()));

      //Close intake
      shooterXbox.povUp()
        .whileTrue(intake.foldCloseIntake());

      /****************************************************************************************************************************/

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
      return autos.getSelected();
    }

    public void setMotorBrake(boolean brake)
    {
     
    }

    public SwerveSubsystem getDrivebase()
    {
      return drivebase;
    }

   
  
}