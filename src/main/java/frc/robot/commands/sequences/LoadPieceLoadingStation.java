// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.*;
import frc.robot.commands.ManipulatorGrab.BehaviorType;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class LoadPieceLoadingStation extends SequentialCommandGroup {


  /**
   * Sets the elevator and wrist for the loading station (height varies for cone vs cube,
   * per current manipulator setting).  Then turns on manipulator motor until a piece is picked up.
   * @param elevator
   * @param wrist
   * @param manipulator
   * @param intake
   * @param log
   */
  public LoadPieceLoadingStation(Elevator elevator, Wrist wrist, Manipulator manipulator, Intake intake, FileLog log) {
    addCommands(
      new FileLogWrite(false, false, "LoadPieceLoadingStation", "Start", log),
      new ParallelCommandGroup(
        new ConditionalCommand(
          new ElevatorWristMoveToUpperPosition(ElevatorPosition.loadingStationCone.value, WristAngle.loadHumanStation.value, elevator, wrist, intake, log), 
          new ElevatorWristMoveToUpperPosition(ElevatorPosition.loadingStationCube.value, WristAngle.loadHumanStation.value, elevator, wrist, intake, log), 
          manipulator::getPistonCone
        ),
        new ManipulatorGrab(ManipulatorConstants.pieceGrabPct, BehaviorType.waitForConeOrCube, manipulator, log)
      ),
      new FileLogWrite(false, false, "LoadPieceLoadingStation", "Finish", log)
    );
  }
}
