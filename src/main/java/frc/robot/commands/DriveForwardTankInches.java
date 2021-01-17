// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveForwardTankInches extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final RomiDrivetrain m_subsystem;

  double inches;
  double speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveForwardTankInches(RomiDrivetrain subsystem, double inches, double speed) {
    this.m_subsystem = subsystem;
    this.inches = inches;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(MessageFormat.format("Started {0}", this.getName()));

    m_subsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(timer.get());
    double leftSpeed=0;
    double rightSpeed=0;
    if(Math.abs(m_subsystem.getLeftDistanceInch()) < inches) {
      leftSpeed = speed;
    }
    if(Math.abs(m_subsystem.getRightDistanceInch()) < inches) {
      rightSpeed = speed;
    }

    m_subsystem.tankDrive(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(MessageFormat.format("Ended {0}", this.getName()));
    m_subsystem.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_subsystem.getLeftDistanceInch()) > inches && Math.abs(m_subsystem.getRightDistanceInch()) > inches;
  }
}
