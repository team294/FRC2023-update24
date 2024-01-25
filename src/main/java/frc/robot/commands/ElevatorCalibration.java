// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.FileLog;

public class ElevatorCalibration extends Command {

  private Elevator elevator;
  private FileLog log;
  private double percentOutput, rampRate;
  private int state;
  private final Timer timer = new Timer();

  /** Creates a new ElevatorCalibration. */
  /**
   * Ramps elevator speed upwards and then downwards, reversing when 
   * 2 inches from top or bottom.
   * @param rampRate Ramp rate in pctOut/second 
   * @param elevator
   * @param log
   */
  public ElevatorCalibration(double rampRate, Elevator elevator, FileLog log) {
    this.elevator = elevator;
    this.log = log;
    this.rampRate = rampRate;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    state = 0;

    elevator.enableFastLogging(true);
    log.writeLog(false, "ElevatorCalibration", "Initialize", "rampRate", rampRate);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currTime = timer.get();

    if (!elevator.encoderCalibrated()) return;

    switch (state) {
      case 0:   // ramp upwards until 10 inches from top
        if (elevator.getElevatorPos() < ElevatorConstants.ElevatorPosition.upperLimit.value - 2.0) {
          percentOutput = MathUtil.clamp(currTime*rampRate, -1.0, 1.0);
          elevator.setElevatorMotorPercentOutput(percentOutput);      
        } else {
          state = 1;
          timer.reset();
          timer.start();
        }
        break;
      case 1:    // stop motor for 2 seconds, then go to state 2
        if (currTime < 2.0) {
          elevator.stopElevator();     
        } else {
          state = 2;
          timer.reset();
          timer.start();
        }
        break;
      case 2:   // ramp downwards until 10 inches from bottom
        if (elevator.getElevatorPos() > ElevatorConstants.ElevatorPosition.lowerLimit.value + 2.0) {
          percentOutput = MathUtil.clamp(-currTime*rampRate, -1.0, 1.0);
          elevator.setElevatorMotorPercentOutput(percentOutput);      
        } else {
          state = 3;
          timer.reset();
          timer.start();
        }
        break;
      default:    // stop motor for 2 seconds, then go to state 0
        if (currTime < 2.0) {
          elevator.stopElevator();     
        } else {
          state = 0;
          timer.reset();
          timer.start();
        }
        break;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
    elevator.enableFastLogging(false);
    log.writeLog(false, "ElevatorCalibration", "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
