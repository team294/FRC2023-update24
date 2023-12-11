// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class IntakeToManipulator extends SequentialCommandGroup {

  /**
   *  Stows Wrist and Elevator, Extends intake and runs intake motor, Runs manipulator motor to pick up from ground
   * @param intake
   * @param elevator
   * @param wrist
   * @param manipulator
   * @param log
   */
  public IntakeToManipulator(Intake intake, Elevator elevator, Wrist wrist, Manipulator manipulator, FileLog log) {
    addCommands(
      new ElevatorWristStow(elevator, wrist, log),
      new IntakePistonSetPosition(true, intake, elevator, log),
      new IntakeSetPercentOutput(IntakeConstants.motor1NominalIntakePct, IntakeConstants.motor2NominalIntakePct, intake, log),
      new ManipulatorSetPercent(ManipulatorConstants.pieceGrabPct, manipulator, log)
    );
  }
}
