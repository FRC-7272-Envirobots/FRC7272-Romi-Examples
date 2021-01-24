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
   * Combines the most sensors in an attempt to do completely straight driving with course correction (if bumped) to ensure robot always drives straight.
   * This command uses encoders to measure the inches to drive, as well as a PID-enable gyro to make sure the robot heading stays at angle 0.
   *
   * @param drivetrain instance of RomiDrivetrain
   * @param inches number of inches to drive
   * @param speed the speed (-1 to 1) to drive, use negative to drive backwards.
   * 
   */
  public DriveForwardInchesGyroStraightPID(RomiDrivetrain drivetrain, double inches, double speed) {
    this.pid = new PIDController(speed, 0, 0);
    
    //within 1 degree of target with 10 data samples
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

    // If the robot has drive its distance but the command hasn't ended then
    // the robot is not at the correct heading and still needs to adjust angle with PID
    if(isDistanceMet()) {
      curr_speed=0;
      // With the Romi the wheels don't turn if speed is less than 1/4 of power.
      if(pid_calc < 0) {
        pid_calc = Math.min(pid_calc, -.25);
      } else {
        pid_calc = Math.max(pid_calc, .25);
      }
    }

    // Offset the speed with pid angle correction adjustments
    double leftSpeed = curr_speed + pid_calc;
    double rightSpeed = curr_speed - pid_calc;
    
    //Debug the speed calculation
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
    //Finish when the driving distance is met and the robot is at the correct heading.
    return isDistanceMet() && pid.atSetpoint();
  }

  private boolean isDistanceMet() {
    return Math.abs(drivetrain.getLeftDistanceInch()) > inches && Math.abs(drivetrain.getRightDistanceInch()) > inches;
  }
}
