// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commandGroups.DriveAndBalanceAutoCommandGroup;
import frc.robot.commandGroups.DriveOverForwardAndBackOnBalance;
import frc.robot.commandGroups.ShiftAndAuto;
import frc.robot.commands.DriveAutoForwardTimedCommand;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveManualCommand;
import frc.robot.commands.TurnAround;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NewGrabber;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.SOLENOID_TEST;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  private static final int XBOX_CONTROLLER = 0;
  private static final int LOGITECH_JOYSTICK = 1;
  private static final Command driveShiftCommand = null;
  //Shuffleboard
  private final ShuffleboardTab autonomousTab = Shuffleboard.getTab("Autonomous");
    private SendableChooser<Command> autoChooser;
  
  // Subsystems
  private final DriveTrain driveTrain = new DriveTrain();
  private final NewGrabber newGrabber = new NewGrabber();

  // Pneumatics
  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private final Solenoid solenoidTest = new Solenoid (PneumaticsModuleType.CTREPCM, SOLENOID_TEST);

  // Camera
  //UsbCamera camera = new UsbCamera("camera", 0);

  // Controllers
  private final XboxController controller =
      new XboxController(XBOX_CONTROLLER);
  private final Joystick joystick = 
      new Joystick (LOGITECH_JOYSTICK);
  // Commands
  //private final DriveManualCommand driveManualCommand = new DriveManualCommand(driveTrain, controller);
  private final DriveManualCommand driveManualCommand = new DriveManualCommand(driveTrain, joystick, controller);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // Set default commands for subsystems
    driveTrain.setDefaultCommand(driveManualCommand);
    // Set up Shuffleboard
    setupAutonomousTab();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(driveTrain::exampleCondition)
    
    // XBox controls
    new JoystickButton(controller, Button.kX.value)
       .onTrue(new InstantCommand(() -> solenoidTest.toggle()));

    //Joystick controls
    new JoystickButton(joystick, Button.kX.value)
      .onTrue(new InstantCommand(() -> solenoidTest.toggle()));

    //Xbox Controller
       new JoystickButton(controller, Axis.kLeftTrigger.value)
       .onTrue(new InstantCommand(() -> newGrabber.in()));
       
       new JoystickButton(controller, Axis.kRightTrigger.value)
       .onTrue(new InstantCommand(() -> newGrabber.out()));
    
   // new POVButton(joystick, Button.kX.value).onTrue(driveShiftCommand);
    

        //new JoystickButton(joystick, Button.kX.value)
        //.onTrue(new InstantCommand(() -> driveTrain.toggle()));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // controller.().whileTrue(driveTrain.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
  private void setupAutonomousTab(){
    autoChooser = new SendableChooser<>();
    SendableRegistry.setName(autoChooser, "Autonomous Command");
    // = Do Nothing
    autoChooser.setDefaultOption("Nothing", null);
      autoChooser.addOption("Drive Forward Timed", new DriveAutoForwardTimedCommand(driveTrain, 2));
      autoChooser.addOption("Shift And Auto Drive", new ShiftAndAuto( driveTrain, solenoidTest));
      autoChooser.addOption("Drive Distance", new DriveDistance(driveTrain, 1, 170000, 0.5 ));
      autoChooser.addOption("Drive Over And Back", new DriveOverForwardAndBackOnBalance(driveTrain, joystick));
      autoChooser.addOption("Drive Over And Balance", new DriveAndBalanceAutoCommandGroup(driveTrain, joystick));
      autoChooser.addOption("Turn Around", new TurnAround(driveTrain, 160, 0.5));
      
      autonomousTab.add(autoChooser)
        .withPosition(0, 0)
        .withSize(2, 1);
    }   //.withPosition(0, 0)
        //.withSize(2, 1);

  }
