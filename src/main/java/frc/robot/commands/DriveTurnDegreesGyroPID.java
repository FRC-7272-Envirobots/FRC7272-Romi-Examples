// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveTurnDegreesGyroPID extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final RomiDrivetrain drivetrain;

  double degrees;
  double speed;
  boolean continuous;
  PIDController pid;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveTurnDegreesGyroPID(RomiDrivetrain drivetrain, double degrees, double speed, boolean continuous) {
    this.pid = new PIDController(speed, 0, 0);
    this.pid.setTolerance(1,10);
    this.drivetrain = drivetrain;
    this.degrees = degrees;
    this.speed = speed;
    this.continuous = continuous;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(MessageFormat.format("Started {0}", this.getName()));
    drivetrain.resetAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curr_angle = drivetrain.getAngle();
    double pid_calc = pid.calculate(curr_angle, degrees)/100;
 
    double speed;
    if(pid_calc < 0) {
      speed = Math.min(pid_calc, -.25);
    } else {
      speed = Math.max(pid_calc, .25);
    }

    drivetrain.arcadeDrive(0, speed);
    System.out.println(MessageFormat.format("current gyro angle, speed {0}\t{1}", curr_angle,speed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(MessageFormat.format("Ended {0}", this.getName()));
    drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(continuous) {
      return false;
    }
    if(pid.atSetpoint()) {
      System.out.println(MessageFormat.format("Ending {0}, current angle: {1}", this.getName(), Math.abs(drivetrain.getAngle())));
      return true;
    }
    return false;
  }
}
