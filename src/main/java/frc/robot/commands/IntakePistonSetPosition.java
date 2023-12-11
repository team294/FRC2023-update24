// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.FileLog;

public class IntakePistonSetPosition extends Command {
  Intake intake;
  Elevator elevator;
  FileLog log;
  boolean deploy;

  /**
   * Sets intake pistion position.
   * @param deploy true = deploy, false = stow
   * @param intake intake subsystem
   * @param elevator elevator subsystem
   * @param log log file
   */
  public IntakePistonSetPosition(boolean deploy, Intake intake, Elevator elevator, FileLog log) {
    this.intake = intake;
    this.elevator = elevator;
    this.deploy = deploy;
    this.log = log;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double elevatorPos = elevator.getElevatorPos();

    if (elevatorPos > ElevatorConstants.boundBottomMain && deploy) {
      log.writeLog(false, "IntakePistonSetPosition", "Initialize", "IntakePiston", (deploy) ? "Deploy" : "Retract", 
        "Elevator pos", elevatorPos, "Command Ok?", "Failure");
    } else {
      intake.setDeployed(deploy);
      log.writeLog(false, "IntakePistonSetPosition", "Initialize", "IntakePiston", (deploy) ? "Deploy" : "Retract", 
        "Elevator pos", elevatorPos, "Command Ok?", "Success");
    }
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
    return true;
  }
}
