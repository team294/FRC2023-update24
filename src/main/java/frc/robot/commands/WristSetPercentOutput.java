// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;

public class WristSetPercentOutput extends Command {

  private double percent;
  private final Wrist wrist;
  private final FileLog log;
  private final boolean fromShuffleboard;

  /**
   * Sets the speed of the wrist
   * @param percent percent output, -1 (down) to +1 (up)
   * @param wrist
   * @param log
   */
  public WristSetPercentOutput(double percent, Wrist wrist, FileLog log) {
    this.percent = percent;
    this.wrist = wrist;
    this.log = log;
    fromShuffleboard = false;

    addRequirements(wrist);
  }

  /**
   * Sets the speed of the wrist from Shuffleboard
   * @param wrist
   * @param log
   */
  public WristSetPercentOutput(Wrist wrist, FileLog log){
    this.wrist = wrist;
    this.log = log;
    fromShuffleboard = true;

    if(SmartDashboard.getNumber("Wrist Output to set", -9999) == -9999) {
      SmartDashboard.putNumber("Wrist Output to set", 0);
    }
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(fromShuffleboard){
      percent = SmartDashboard.getNumber("Wrist Output to set", 0);
    }
    wrist.setWristMotorPercentOutput(percent);
    log.writeLog(false, "WristSetPercentOutput", "Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
