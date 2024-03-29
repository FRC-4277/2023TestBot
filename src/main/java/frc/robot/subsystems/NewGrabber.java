// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.GrabConstants.*;


/** Add your docs here. */
public class NewGrabber extends SubsystemBase {
    private final WPI_TalonSRX top = new WPI_TalonSRX(TOP);
    private final WPI_TalonSRX bottom = new WPI_TalonSRX(BOTTOM);
 /** Creates a new Grabber. */
 public NewGrabber() {}

 @Override
 public void periodic() {
   // This method will be called once per scheduler run
 }
 public void in() {
   move(1);
 }
 public void out(){
   move(-1);
 }
 public void move(int direction){
   top.set(0.5 * direction);
   bottom.set(-0.5 * direction);
 }
}