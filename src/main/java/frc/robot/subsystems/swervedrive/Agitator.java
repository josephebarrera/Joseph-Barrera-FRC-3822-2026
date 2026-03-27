package frc.robot.subsystems.swervedrive;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Agitator extends SubsystemBase
{
    //Motor
    SparkMax agitator = new SparkMax(15, MotorType.kBrushless);

    //************************************************* Commands *************************************************/
    public Command funnelForward()
    {
        return Commands.run(()->
        {
            System.out.println("Setting funnel speed to ");
            agitator.set(-100.0);
        });
    }

    public Command funnelReverse()
    {
        return Commands.runOnce(()->
        {
            agitator.set(100.0);
        });
    }

    public Command funnelStop() 
    {
        return Commands.runOnce(()->
        {
            agitator.set(0.0);
        });
    }
    //************************************************************************************************************/

    //************************************************* Auto Commands ********************************************/
    public Command funnelForwardAuto()
    {
        return Commands.runOnce(()->
        {
            agitator.set(-100);
        });
    }

}