package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import swervelib.SwerveInputStream;
import frc.robot.subsystems.swervedrive.VisionSubsystem;
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

    //Give SmartDashboard the ability to choose Autos
    private final SendableChooser<Command> autos = new SendableChooser<>();

    /**
    * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
    */
    public SwerveInputStream driveInputStream = SwerveInputStream.of(
    drivebase.getSwerveDrive(),
    () -> driverXbox.getLeftY(),
    () -> driverXbox.getLeftX())
    .withControllerRotationAxis(()-> driverXbox.getRightX())
    .deadband(Constants.OperatorConstants.DEADBAND)
    .scaleTranslation(0.5) //Originally 0.5
    .allianceRelativeControl(false);
      
    /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
    public RobotContainer()
    {

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
      
    }

    private void configureBindings()
    {
      /*********************************************************** Driver Commands ***************************************************/

      //Zero the gyro
      driverXbox.b()
        .onTrue(Commands.runOnce(drivebase::zeroGyroAllianceAware));

      Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveInputStream);
     
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      
      /*******************************************************************************************************************************/  
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