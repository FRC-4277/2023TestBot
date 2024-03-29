// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

public class DriveOnChargeStationCommand extends SequentialCommandGroup {

  // List commands here sequentially
  public DriveOnChargeStationCommand(DriveTrain driveTrain) { // List commands here sequentially
    addCommands(new DriveDistance(driveTrain, 1, 101000, 0.6));
  }

}
