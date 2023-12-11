/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.RobotPreferences;

public class StickyFaultsClear extends Command {

  private FileLog log;

  /**
   * Clears the sticky faults in RobotPreferences.
   * <p> Note:  This command can run while the robot is disabled.
   * @param log
   */
  public StickyFaultsClear(FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.log = log;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotPreferences.clearStickyFaults(log);
    log.writeLog(false, "ClearStickyFaults", "Init");
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

  /**
   * Whether the given command should run when the robot is disabled. Override to return true if the
   * command should run when disabled.
   * @return whether the command should run when the robot is disabled
   */
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
