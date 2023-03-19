// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAutoForwardTimedCommand;
import frc.robot.commands.DriveShiftCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShiftAndAuto extends SequentialCommandGroup {
  private final DriveTrain driveTrain;
  private final Solenoid solenoid;

  /** Creates a new ScoreAndDrive. */
  public ShiftAndAuto(DriveTrain driveTrain, Solenoid solenoid) {
    this.addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.solenoid = solenoid;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveAutoForwardTimedCommand(driveTrain, 1),
      new DriveShiftCommand(driveTrain, solenoid)
    );
  }
}
