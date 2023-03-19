// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveTrainConstants.BACK_LEFT;
import static frc.robot.Constants.DriveTrainConstants.BACK_RIGHT;
import static frc.robot.Constants.DriveTrainConstants.FRONT_LEFT;
import static frc.robot.Constants.DriveTrainConstants.FRONT_RIGHT;
import static frc.robot.Constants.DriveTrainConstants.COUNTS_PER_REVOLUTION;
import static frc.robot.Constants.DriveTrainConstants.GEAR_RATIO;
import static frc.robot.Constants.DriveTrainConstants.WHEEL_DIAMETER;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

import static frc.robot.Constants.DriveTrainConstants.*;

public class DriveTrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonFX leftFollower = new WPI_TalonFX(FRONT_LEFT);
  private final WPI_TalonFX leftLead = new WPI_TalonFX (BACK_LEFT);
  private final WPI_TalonFX rightFollower = new WPI_TalonFX(FRONT_RIGHT);
  private final WPI_TalonFX rightLead = new WPI_TalonFX(BACK_RIGHT);

  private MotorControllerGroup leftGroup = new MotorControllerGroup(leftFollower, leftLead);
  private MotorControllerGroup rightGroup = new MotorControllerGroup(rightFollower, rightLead);

  private final DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

 // private final SlewRateLimiter speedLimiter = new SlewRateLimiter(0.8);
 // private final SlewRateLimiter twistLimiter = new SlewRateLimiter(0.8);


  private AHRS navx = new AHRS(SerialPort.Port.kMXP);
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public DriveTrain(){
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    leftLead.configAllSettings(configs);
    rightLead.configAllSettings(configs);

    leftLead.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    rightLead.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

    leftLead.setNeutralMode(NeutralMode.Brake);
    rightLead.setNeutralMode(NeutralMode.Brake);
  }


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  //public void driveController(XboxController controller){
   //drive.arcadeDrive(controller.getRightX(), controller.getRightY());
 // }
  public void driveJoystick(Joystick joystick ){
    //drive.arcadeDrive((Math.pow(joystick.getZ(),3))*0.9, Math.pow(joystick.getY(),3)*0.9);
    
    //TODO:  Try this
    //drive.arcadeDrive(speedLimiter.calculate(joystick.getZ()), twistLimiter.calculate(joystick.getY()));
    
    drive.arcadeDrive(joystick.getZ(), joystick.getY());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Object toggle() {
    return null;
  }
  public void driveTimed(int direction, double speed) {
    System.out.println("start driveTimed direction " + direction);
    System.out.println("start driveTimed speed " + speed);
    double howToDrive = direction * speed;
    drive.tankDrive(howToDrive, -howToDrive);
  }
  public void driveDistance(int direction, double speed) {
    System.out.println(" start DriveDistance direction" + direction);
    System.out.println("start driveDistance speed" + speed);
    double howToDrive = direction * speed;
    drive.tankDrive(howToDrive, -howToDrive);
  }
  public void turnToAngle(double speed, double requestedDegrees) {
    System.out.println("start turn to angle");
    drive.tankDrive(speed, 0);
  }

  /* Set power to the drivetrain motors */
  public void driveForBalance(int direction, double leftPercentPower, double rightPercentPower) {
    leftGroup.set(direction * leftPercentPower);
    rightGroup.set(direction * rightPercentPower);
  }

  public void stop() {
    drive.stopMotor();
  }

public void reportToShuffleboard(Joystick joystick) {
  //SmartDashboard.putNumber("Joystick Z Value", joystick.getZ());
  //SmartDashboard.putNumber("Joystick Y Value", joystick.getY());
  SmartDashboard.putNumber("NavX yaw", navx.getYaw());
  SmartDashboard.putNumber("NavX pitch", navx.getPitch());
  SmartDashboard.putNumber("NavX roll", navx.getRoll());
  SmartDashboard.putNumber("NavX angle", navx.getAngle());
  //SmartDashboard.putNumber("Right Position", getRightPosition());
  //SmartDashboard.putNumber("Left Position", getLeftPosition());
  SmartDashboard.putNumber("Right Motor Output Percent", getRightMotorOutputPercent());
  SmartDashboard.putNumber("Left Motor Output Percent", getLeftMotorOutputPercent());
  SmartDashboard.putNumber("Right Selected Sensor Position", getRightSelectedSensorPosition());
  SmartDashboard.putNumber("Left Selected Sensor Position", getLeftSelectedSensorPosition());
}

  public void zeroGyro() {
    System.out.println("NavX Connected: " + navx.isConnected());
    navx.reset();
  }
  public double getYaw() {
    return navx.getYaw();
  }
  public double getPitch() {
    return navx.getPitch();
  }
  public double getRoll() {
    return navx.getRoll();
  }
  public double getAngle() {
    return navx.getAngle();
  }
  public double getRightPosition() {
    return rightLead.getActiveTrajectoryPosition();
  }
  public double getLeftPosition() {
    return leftLead.getActiveTrajectoryPosition();

  }

  public double getRightMotorOutputPercent() {
    return rightLead.getMotorOutputPercent();
  }
  public double getLeftMotorOutputPercent() {
    return leftLead.getMotorOutputPercent();
    
  }
  public void setRightSelectedSensorPosition(double position) {
    ErrorCode error = rightLead.setSelectedSensorPosition(position);
    System.out.println("Set to zero " + error);  

  }
  public double getRightSelectedSensorPosition() {
    return rightLead.getSelectedSensorPosition();
  }
  public double getLeftSelectedSensorPosition(){
    return leftLead.getSelectedSensorPosition();
    
  }
  public double nativeUnitsToDistanceMeters(double sensorCounts) {
    double motorRotations = (double)sensorCounts / COUNTS_PER_REVOLUTION;
    double wheelRotations = motorRotations/GEAR_RATIO;
    double positionInMeters = wheelRotations * (2*Math.PI * Units.inchesToMeters(wheelRotations));
    return positionInMeters;
  }

  public double distanceToNativeUnits(double distanceInInches) {
    double wheelRotations =  distanceInInches / (Math.PI * (WHEEL_DIAMETER));
    double motorRotations = GEAR_RATIO * wheelRotations;
    int sensorCounts = (int) motorRotations * COUNTS_PER_REVOLUTION;
    return sensorCounts;

  }
    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
      navx.reset();
    }
  
    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    public double getHeading() {
      double heading = Math.IEEEremainder(navx.getAngle(), 360) * (DriveTrainConstants.kGyroReversed ? -1.0 : 1.0);
      System.out.println("Heading" + heading );
      return heading;
    }
  
    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
      return navx.getRate() * (DriveTrainConstants.kGyroReversed ? -1.0 : 1.0);
    }
}