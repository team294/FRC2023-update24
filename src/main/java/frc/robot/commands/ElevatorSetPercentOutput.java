/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.FileLog;

public class ElevatorSetPercentOutput extends Command {

  private final Elevator elevator;
  private final FileLog log;

  private final boolean fromShuffleboard;
  
  private double percentOutput;

  public ElevatorSetPercentOutput(double percentOutput, Elevator elevator, FileLog log) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.elevator = elevator;
    this.log = log;
    this.percentOutput = percentOutput;
    fromShuffleboard = false;
    addRequirements(elevator);
  }

  public ElevatorSetPercentOutput(Elevator elevator, FileLog log) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.elevator = elevator;
    this.log = log;
    fromShuffleboard = true;
    addRequirements(elevator);

    if(SmartDashboard.getNumber("Elevator Percent", -9999) == -9999) {
      SmartDashboard.putNumber("Elevator Percent", 0);
    }
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    if(fromShuffleboard){
      percentOutput = SmartDashboard.getNumber("Elevator Percent", 0);
    }
    elevator.setElevatorMotorPercentOutput(percentOutput);
    log.writeLog(false, "ElevatorSetPercentOutput", "Percent", percentOutput);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
  }

}
