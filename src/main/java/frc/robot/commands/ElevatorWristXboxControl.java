// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;

public class ElevatorWristXboxControl extends Command {
  private final CommandXboxController xboxController;
  private final Elevator elevator;
  private final Wrist wrist;
  private final FileLog log;

  /**
   * Controls the elevator and wrist using the left and right XBox Controller joysticks
   * @param xboxController
   * @param elevator
   * @param wrist
   * @param log
   */
  public ElevatorWristXboxControl(CommandXboxController xboxController, Elevator elevator, Wrist wrist, FileLog log) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.xboxController = xboxController;
    this.log = log;

    addRequirements(elevator, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "ElevatorWristXboxControl", "Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elevatorPct = xboxController.getLeftX();
    double wristPct = xboxController.getRightX();
    if (Math.abs(elevatorPct)<OIConstants.manualElevatorDeadband) elevatorPct=0;
    if (Math.abs(wristPct)<OIConstants.manualWristDeadband) wristPct=0;
    wristPct *= WristConstants.maxPercentOutput;
    elevatorPct *= ElevatorConstants.maxManualPercentOutput;

    log.writeLog(false, "ElevatorWristXboxControl", "Execute", "Left Xbox", elevatorPct, "Right Xbox", wristPct);

    // Only run one element at a time (elevator or wrist)
    // Run whichever is being set to a higher value
    if (Math.abs(elevatorPct)>=Math.abs(wristPct)) {
      wristPct = 0.0;
    } else {
      elevatorPct = 0.0;
    }

    elevator.setElevatorMotorPercentOutput(elevatorPct);
    wrist.setWristMotorPercentOutput(wristPct);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
    wrist.stopWrist();
    
    log.writeLog(false, "ElevatorWristXboxControl", "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
