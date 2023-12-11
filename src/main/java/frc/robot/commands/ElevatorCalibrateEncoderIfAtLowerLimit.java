/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.FileLog;

public class ElevatorCalibrateEncoderIfAtLowerLimit extends Command{
  private final Elevator elevator;
  private final FileLog log;

  public ElevatorCalibrateEncoderIfAtLowerLimit(Elevator elevator, FileLog log) {
    this.elevator = elevator;
    this.log = log;
    addRequirements(elevator);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    elevator.checkAndZeroElevatorEnc();
    log.writeLog(false, "ElevatorCalibrateEncoderIfAtLowerLimit", "Calibrating at lower limit");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }
}
