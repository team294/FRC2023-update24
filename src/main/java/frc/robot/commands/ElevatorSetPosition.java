/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.FileLog;

public class ElevatorSetPosition extends Command {

  private final Elevator elevator;
  private final FileLog log;

  private double target;

  private double toleranceCounter;
  private final boolean fromShuffleboard;


  /**
   * Moves elevator to target height
   * @param target target height in inches, per ElevatorConstants.ElevatorPosition
   * @param elevator elevator subsystem
   * @param log log subsystem
   */
  public ElevatorSetPosition(double target, Elevator elevator, FileLog log) {
    this.elevator = elevator;
    this.log = log;
    this.target = target;
    fromShuffleboard = false;

    addRequirements(elevator);
  }

  /**
   * Moves elevator to target position
   * @param position target ElevatorPosition (see Constants)
   * @param elevator elevator subsystem
   * @param log log subsystem
   */
  public ElevatorSetPosition(ElevatorPosition position, Elevator elevator, FileLog log) {
    this(position.value, elevator, log);
  }

  /**
   * Moves elevator to target height from shuffleboard
   * @param elevator elevator subsystem
   * @param log log subsystem
   */
  public ElevatorSetPosition(Elevator elevator, FileLog log) {
    this.elevator = elevator;
    this.log = log;
    fromShuffleboard = true;
    if(SmartDashboard.getNumber("Elevator Position", -9999) == -9999) {
      SmartDashboard.putNumber("Elevator Position", 0);
    }
    addRequirements(elevator);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    if(fromShuffleboard){
      target = SmartDashboard.getNumber("Elevator Position", 0);
    }
    log.writeLog(false, "ElevatorSetPosition", "Initialize", "Target Position", target);

    toleranceCounter = 0;
    elevator.setProfileTarget(target);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    log.writeLog(false, "ElevatorSetPosition", "End", "Target Position", target);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if (!elevator.encoderCalibrated() ||         // End immediately if encoder can't read
      Math.abs(elevator.getElevatorPos() - target) <= 0.5) {
        toleranceCounter++;
        log.writeLog(false, "ElevatorSetPosition", "Within Tolerance", "Target Position", target, "Position", elevator.getElevatorPos(), "Tol count", toleranceCounter);
    }
    return (toleranceCounter > 5);
  }

}
