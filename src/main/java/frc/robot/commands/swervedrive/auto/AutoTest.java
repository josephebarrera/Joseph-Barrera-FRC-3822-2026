// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Turret;
import frc.robot.subsystems.swervedrive.Shooter;
import frc.robot.subsystems.swervedrive.Agitator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTest extends SequentialCommandGroup {
  /** Creates a new AutoTest. */
  public AutoTest(SwerveSubsystem objSwerve, Turret objTurret, Shooter objShooter, Agitator objAgitator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // objSwerve.driveForward().withTimeout(2.0),
      // objTurret.
      // objShooter.shootForward().withTimeout(0.25),
      
    );
  }
}
