package frc.robot.subsystems.swervedrive;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;

public class Intake extends SubsystemBase
{
    //Motors
    SparkMax intake = new SparkMax(10, MotorType.kBrushless);
    SparkMax armJoint = new SparkMax(29, MotorType.kBrushless);

    //Encoder
    private final RelativeEncoder armEncoder = armJoint.getEncoder();

    //Add a target open position
    private final double OPEN_POSITION = 30; //TUNE LATER

    //Add a target closed position
    private final double CLOSED_POSITION = 0;

    //************************************************* Commands *************************************************/
    public Command spinIntakeForward() 
    {
        return Commands.run(()->
        {
            intake.set(-100.0);
        })
        .finallyDo(()->
        {
            intake.set(0);
        });
    }

     public Command stopIntake()
    {
        return Commands.runOnce(()->
        {
            intake.set(0.0);
        });
    }

    public Command foldOpenIntake()
    {
        return Commands.run(()->
        {
            armJoint.set(1); //slow and safely
            System.out.println(armEncoder.getPosition()); //will help us tune 
        })
        .until(()-> armEncoder.getPosition() >= OPEN_POSITION)
        .finallyDo(()-> armJoint.set(0));
    }

    public Command foldCloseIntake()
    {
        return Commands.run(()->
        {
            armJoint.set(-1); //reverse the direction
        })
        .until(()->armEncoder.getPosition() <= CLOSED_POSITION)
        .finallyDo(()->armJoint.set(0));
    }

    public Command spinIntakeReverse() 
    {
        return Commands.runOnce(()->
        {
            intake.set(100.0);
        });
    }

}