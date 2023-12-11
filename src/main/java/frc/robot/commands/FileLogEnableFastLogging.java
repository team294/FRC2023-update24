/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.utilities.Loggable;
import frc.robot.utilities.FileLog;

public class FileLogEnableFastLogging extends Command {
  private boolean enabled;
  private Loggable subsystem;
  private FileLog log;

  /**
   * Changes logging cycle for a subsystem
   * @param enabled true is enabled for every cycle; false follows normal logging cycles
   * @param subsystem subsystem which logging cycle is changed
   */
  public FileLogEnableFastLogging(boolean enabled, Loggable subsystem, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.enabled  = enabled;
    this.subsystem = subsystem;
    this.log = log;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.enableFastLogging(enabled);
    log.writeLog(false, "LogEnableFastLogging", "initialize", "Fast Logging", enabled);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
