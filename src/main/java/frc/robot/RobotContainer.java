// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commandgroups.DriveASquare;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DriveForwardInches;
import frc.robot.commands.DriveForwardInchesGyroStraightPID;
import frc.robot.commands.DriveForwardTimed;
import frc.robot.commands.DriveTurnDegrees;
import frc.robot.commands.DriveTurnDegreesGyro;
import frc.robot.commands.DriveTurnDegreesGyroPID;
import frc.robot.commands.DriveTurnNetworkTableAnglePID;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();

  // Controller & Joysticks
  private final XboxController controller = new XboxController(0);
  private final Joystick keyboard = new Joystick(1);

  // The network table entries used across commands
  private final NetworkTableEntry visionAngle;
  private final NetworkTableEntry visionObjects;

  //Default Commands
  private final DefaultDrive defaultDrive = new DefaultDrive(controller, m_romiDrivetrain);

  //Commands
  private final DriveForwardTimed forwardTimed = new DriveForwardTimed(m_romiDrivetrain, 3, .5);
  private final DriveForwardInches forward3in = new DriveForwardInches(m_romiDrivetrain, 3, .5);
  private final DriveForwardInches backward3in = new DriveForwardInches(m_romiDrivetrain, 3, -.5);
  private final DriveTurnDegrees right90degrees = new DriveTurnDegrees(m_romiDrivetrain, 90, .5);
  private final DriveTurnDegrees left90degrees = new DriveTurnDegrees(m_romiDrivetrain, -90, .5);
  private final DriveASquare squareDrive = new DriveASquare(m_romiDrivetrain);
  private final DriveTurnDegreesGyro gyroTurnRight = new DriveTurnDegreesGyro(m_romiDrivetrain, 90, .25);
  private final DriveTurnDegreesGyro gyroTurnLeft = new DriveTurnDegreesGyro(m_romiDrivetrain, -90, .25);
  private final DriveTurnDegreesGyroPID gyroPidTurnRightOnce = new DriveTurnDegreesGyroPID(m_romiDrivetrain, 90, .7, false);
  private final DriveTurnDegreesGyroPID gyroPidTurnLeftOnce = new DriveTurnDegreesGyroPID(m_romiDrivetrain, -90, .7, false);
  private final DriveTurnDegreesGyroPID gyroPidTurnRightCont = new DriveTurnDegreesGyroPID(m_romiDrivetrain, 90, .5, true);
  private final DriveForwardInchesGyroStraightPID driveForwardInchesGyro = new DriveForwardInchesGyroStraightPID(m_romiDrivetrain, 24, .7);
  private final DriveTurnNetworkTableAnglePID driveTurnNetworkTable;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Get the network table entries used in the robot program.
    // In this example I'm using https://www.chiefdelphi.com/t/romi-vision/390540.
    // Just upload the jar from the build directory on github into http://wpilibpi.local/ as the Vision Application.
    // I don't have reflective tape on hand and tested (not ideal) with a piece of printer paper.
    // Done in constructor because NetworkTableEntry is just a reference to the entry, use get method in commands to retrieve values.
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable visionTargetData = inst.getTable("targetData");
    this.visionAngle = visionTargetData.getEntry("drCorr");
    this.visionObjects = visionTargetData.getEntry("numberImages");
    
    // Instantiate commands using network table here (too keep all fields as finals)
    this.driveTurnNetworkTable = new DriveTurnNetworkTableAnglePID(m_romiDrivetrain, visionAngle, visionObjects, .7, true);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Keyboard buttoh bindings
    JoystickButton keyZ = new JoystickButton(keyboard, 1);
    JoystickButton keyX = new JoystickButton(keyboard, 2);
    JoystickButton keyC = new JoystickButton(keyboard, 3);
    JoystickButton keyV = new JoystickButton(keyboard, 4);
    JoystickButton keyB = new JoystickButton(keyboard, 5);
    JoystickButton keyG = new JoystickButton(keyboard, 6);
    keyZ.whenPressed(forwardTimed);
    keyX.whenPressed(left90degrees);
    keyC.whenPressed(right90degrees);
    keyV.whenPressed(gyroTurnLeft);
    keyB.whenPressed(gyroTurnRight);
    keyG.whenPressed(driveTurnNetworkTable);

    //Xbox 360 controller button bindings
    JoystickButton buttonA = new JoystickButton(controller, 1);
    JoystickButton buttonB = new JoystickButton(controller, 2);
    JoystickButton buttonX = new JoystickButton(controller, 3);
    JoystickButton buttonY = new JoystickButton(controller, 4);
    JoystickButton buttonL = new JoystickButton(controller, 5);
    JoystickButton buttonR = new JoystickButton(controller, 6);
    JoystickButton buttonBack = new JoystickButton(controller, 7);
    JoystickButton buttonStart = new JoystickButton(controller, 8);
    buttonA.whenPressed(forward3in);
    buttonB.whenPressed(gyroPidTurnRightOnce);
    buttonX.whenPressed(gyroPidTurnLeftOnce);
    buttonY.whenPressed(backward3in);
    buttonL.whenPressed(gyroPidTurnRightCont);
    buttonR.whenPressed(driveForwardInchesGyro);
    //buttonBack.whenPressed();
    //buttonStart.whenPressed();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return forward3in;
  }

  public Subsystem getDrivetrain() {
    return m_romiDrivetrain;
  }

  public Command getDefaultDrive() {
    return defaultDrive;
  }

  public void printGyroOffsets() {
    m_romiDrivetrain.printGyroOffsets();
  }

}
