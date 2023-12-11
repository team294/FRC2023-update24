// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

public class DriveStop extends Command {

  private DriveTrain drivetrain;
  private FileLog log;

  /**
   * Stops all drivetrain motors
   * @param drivetrain Drivetrain subsystem
   */
  public DriveStop(DriveTrain drivetrain, FileLog log) {
    this.drivetrain = drivetrain;
    this.log = log;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "DriveStop", "Stopping Motors");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.stopMotors();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
