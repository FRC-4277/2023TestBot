// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveDistance extends CommandBase {
  public final DriveTrain driveTrain;
  public final int direction;
  public final double distance;
  public final double percent;
  public double initialDistance;
  /** Creates a new DriveDistance. */
  public DriveDistance(DriveTrain driveTrain, int direction, double distance, double percent) {
    

    //TODO:  convert from inches to pulses for distance
    this.driveTrain = driveTrain;
    this.direction = direction;
    this.distance = distance;
    //this.distance = driveTrain.nativeUnitsToDistanceMeters(distance);
    this.percent = percent;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initialize");
    driveTrain.setRightSelectedSensorPosition(0);
    initialDistance = Math.abs(driveTrain.getRightSelectedSensorPosition());
    System.out.println("Right initial distance: " + initialDistance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.driveDistance(direction, percent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    driveTrain.reportToShuffleboard(null);
    double desiredPosition = initialDistance + distance;
    // double currentPosition =  (Math.abs(driveTrain.getRightSelectedSensorPosition())) ;
    // boolean shouldStop = desiredPosition > currentPosition;

   double currentPosition = Math.abs(-1* driveTrain.getRightSelectedSensorPosition()) ;
   boolean shouldStop = currentPosition >= distance;
  
    System.out.println(" Asked for Distance " + distance);
    System.out.println(" Desired Distance " + desiredPosition);
    System.out.println(" Asked for Current position " + currentPosition);
    System.out.println(" Current position " + currentPosition);
    System.out.println(" Should stop driving " + shouldStop);
    return shouldStop;
  }
}
