// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

//Used as the default command for RomiDrivetrain for teleop driving.
public class DefaultDrive extends CommandBase {
  private XboxController controller;
  private RomiDrivetrain drivetrain;

  // I added DigitalInput 0 input as an easy way to troubleshoot Romi connection issues.
  // Sometimes Romi will disconnect without clearly showing this in simulation
  // di will default to High when not connected, and when the romi is connected, di0 should be Low.
  // di0 corresponds to Button A on the Romi which you can press to confirm the connection & simulation is responsive.
  private DigitalInput d0 = new DigitalInput(0);

  /** Creates a new DefaultDrive. */
  public DefaultDrive(XboxController controller, RomiDrivetrain drivetrain) {
    this.controller = controller;
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // For my controller mappings, this corresponds to the right joystick moving forward/back and left joystick left/right.
    drivetrain.arcadeDrive(controller.getRawAxis(5), controller.getRawAxis(0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
