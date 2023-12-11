// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.ManipulatorGrab.BehaviorType;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class LoadPieceConveyor extends SequentialCommandGroup {


  /**
   * Sets the elevator and wrist for intake from conveyor.  Then turns on manipulator motor until a piece is picked up.
   * @param elevator
   * @param wrist
   * @param manipulator
   * @param conveyor
   * @param log
   */
  public LoadPieceConveyor(Elevator elevator, Wrist wrist, Manipulator manipulator, Conveyor conveyor, FileLog log) {
    addCommands(
      new ElevatorWristStow(elevator, wrist, log),
      new ConveyorMove(0.3, conveyor, log),
      new ManipulatorGrab(ManipulatorConstants.pieceGrabPct, BehaviorType.waitForConeOrCube, manipulator, log),
      new ConveyorMove(0, conveyor, log)
    );
  }
}
