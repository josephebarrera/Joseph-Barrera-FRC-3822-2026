package frc.robot.subsystems.swervedrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Actuator extends SubsystemBase
{

    //Create thr servo
    private final Servo actuator = new Servo(0);

    // Positions (0.0 to 1.0)
    private static final double UP_POSITION = .9;
    private static final double DOWN_POSITION = 0.4;

    public Actuator()
    {
    }

    /********************* Basic control *********************/

    public void goUp()
    {
        actuator.set(UP_POSITION);
        System.out.println("going up");
    }

    public void goDown()
    {
        actuator.set(DOWN_POSITION);
        System.out.println("going down");
    }

    public void stop()
    {
        actuator.setDisabled(); // stops sending signal (optional)
    }

    /********************* Commands *********************/

    public Command goUpCommand()
    {
        return Commands.runOnce(() -> goUp(), this);
    }

    public Command goDownCommand()
    {
        return Commands.runOnce(() -> goDown(), this);
    }

    public Command stopCommand()
    {
        return Commands.runOnce(() -> stop(), this);
    }

}