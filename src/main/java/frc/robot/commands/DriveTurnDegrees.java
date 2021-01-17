// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveTurnDegrees extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private static final double ROMI_DIAMETER = 5.75; //Based on measuring the outer edge of the wheels
  private static final double ROMI_CIRCUMFERENCE = Math.PI * ROMI_DIAMETER;

  private final RomiDrivetrain m_subsystem;

  double degrees;
  double speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveTurnDegrees(RomiDrivetrain subsystem, double degrees, double speed) {
    this.m_subsystem = subsystem;
    this.degrees = degrees;
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
    m_subsystem.arcadeDrive(0, degrees < 0 ? speed * -1 : speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(MessageFormat.format("Ended {0}", this.getName()));
    m_subsystem.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distanceToTravel = Math.abs(ROMI_CIRCUMFERENCE * degrees/360);
    return Math.abs(m_subsystem.getLeftDistanceInch()) > distanceToTravel && Math.abs(m_subsystem.getRightDistanceInch()) > distanceToTravel;
  }
}
