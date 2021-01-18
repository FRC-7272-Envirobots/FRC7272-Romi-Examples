// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;

import java.text.MessageFormat;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveForwardInchesGyroStraightPID extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final RomiDrivetrain drivetrain;

  PIDController pid;
  double inches;
  double speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveForwardInchesGyroStraightPID(RomiDrivetrain drivetrain, double inches, double speed) {
    this.pid = new PIDController(speed, 0, 0);
    this.pid.setTolerance(1, 10);
    this.drivetrain = drivetrain;
    this.inches = inches;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(MessageFormat.format("Started {0}", this.getName()));
    drivetrain.resetAngle();
    drivetrain.resetEncoders();
    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pid_calc = pid.calculate(drivetrain.getAngle(), 0)  / 100;
    double curr_speed  = speed;
    if(isDistanceMet()) {
      curr_speed=0;

      if(pid_calc < 0) {
        pid_calc = Math.min(pid_calc, -.25);
      } else {
        pid_calc = Math.max(pid_calc, .25);
      }

    }
    double leftSpeed = curr_speed - pid_calc;
    double rightSpeed = curr_speed + pid_calc;
    System.out.println(MessageFormat.format("LeftSpeed: {0}, RightSpeed: {1}", leftSpeed, rightSpeed));
    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(MessageFormat.format("Ended {0}", this.getName()));
    drivetrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDistanceMet() && pid.atSetpoint();
  }

  private boolean isDistanceMet() {
    return Math.abs(drivetrain.getLeftDistanceInch()) > inches && Math.abs(drivetrain.getRightDistanceInch()) > inches;
  }
}
