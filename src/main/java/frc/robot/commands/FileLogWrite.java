/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.FileLog;

public class FileLogWrite extends Command {
  /**
   * Command to write something to the fileLog
   * To be called in commandGroups (since we can't use the method)
   */
  FileLog log;
  boolean echo;
  boolean logWhenDisabled;
  String subsystemOrCommand;
  String event;
  Object[] paramArray;

  public FileLogWrite(boolean echo, boolean logWhenDisabled, String subsystemOrCommand, String event, FileLog log, Object... paramArray) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.echo = echo;
    this.logWhenDisabled = logWhenDisabled;
    this.subsystemOrCommand = subsystemOrCommand;
    this.event = event;
    this.paramArray = paramArray;
    this.log = log;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(echo) {
      log.writeLogEcho(logWhenDisabled, subsystemOrCommand, event, paramArray);
    } else {
      log.writeLog(logWhenDisabled, subsystemOrCommand, event, paramArray);
    }
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
