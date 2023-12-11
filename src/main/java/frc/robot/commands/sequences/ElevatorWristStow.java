// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class ElevatorWristStow extends SequentialCommandGroup {


  /**
   * Stows the elevator and wrist in the load-intake position.  Appropriately moves the
   * wrist and elevator regardless of starting configuration
   * @param elevator
   * @param wrist
   * @param log
   */
  public ElevatorWristStow(Elevator elevator, Wrist wrist, FileLog log) {
    addCommands(
      // If elevator is higher than a little into main, then put wrist back before lowering
      new ConditionalCommand(
        new WristSetAngle(WristAngle.elevatorMoving, wrist, log),
        new WaitCommand(0.01), 
        () -> (elevator.getElevatorPos() >= ElevatorPosition.belowScoringPegs.value)
      ),

      // lower elevator.  Continue code when safe to start moving wrist (past cone pegs)
      new ElevatorSetPosition(ElevatorPosition.bottom, elevator, log)
        .until( () -> elevator.getElevatorPos() <= ElevatorPosition.belowScoringPegs.value),

      // start moving wrist.  Continue code when safe to stow wrist (elevator is down)
      new WristSetAngle(WristAngle.startConfig, wrist, log)        
        .until( () -> elevator.getElevatorPos() < ElevatorConstants.boundBottomMain),

      // move wrist to stow position
      new WristSetAngle(WristAngle.loadIntake, wrist, log)
    );
  }
}
