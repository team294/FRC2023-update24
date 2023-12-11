// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.Constants.WristConstants.WristRegion;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class ElevatorWristMoveToUpperPosition extends SequentialCommandGroup {
  // public enum ScorePosition {
  //   scoreLow,
  //   scoreMid,
  //   scoreHigh
  // }

  /**
   * Moves the elevator and wrist to an upper position.  Appropriately moves the
   * wrist and elevator regardless of starting configuration to get to the requested configuration.
   * @param elevatorPosition target height in inches, per ElevatorConstants.ElevatorPosition
   * @param wristAngle target angle in degrees.  (0 = horizontal in front of robot, + = up, - = down)
   * @param elevator
   * @param wrist
   * @param intake
   * @param log
   */
  public ElevatorWristMoveToUpperPosition(double elevatorPosition, double wristAngle, Elevator elevator, Wrist wrist, Intake intake, FileLog log) {
    // if (elevatorPosition<ElevatorConstants.boundBottomMain) {
    //   elevatorPosition = ElevatorConstants.boundBottomMain + 1.0;
    // }
    if (wristAngle<WristConstants.boundBackMain) {
      wristAngle = WristConstants.boundBackMain + 5.0;
    }

    addCommands(
      new FileLogWrite(false, false, "ElevatorWristMoveToUpperPosition", "Start", log),

      // Only extend the elevator if the intake is not deployed
      new ConditionalCommand(
        
        new SequentialCommandGroup(
          // If wrist is in back region, then move wrist into main region
          // so that the elevator can move up
          new ConditionalCommand(
            new WristSetAngle(WristAngle.elevatorMoving, wrist, log)
              .until( () -> wrist.getWristAngle() >= WristConstants.boundBackMain+5.0), 
            new WaitCommand(0.01), 
            () -> (wrist.getWristRegion() == WristRegion.back)
          ),

          Commands.deadline(
            // move wrist to safe travel position
            new WristSetAngle(WristAngle.elevatorMoving, wrist, log),
            // start moving elevator up, but no higher than the lower peg
            new ElevatorSetPosition(Math.min(elevatorPosition, ElevatorPosition.belowScoringPegs.value), elevator, log)
          ),

          // move elevator to final position
          new ElevatorSetPosition(elevatorPosition, elevator, log),

          // move wrist to final position
          new WristSetAngle(wristAngle, wrist, log),

          new FileLogWrite(false, false, "ElevatorWristMoveToUpperPosition", "Finish", log)
          ),
        
        // Don't extend if the intake is deployed
        new FileLogWrite(false, false, "ElevatorWristMoveToUpperPosition", "Intake deployed", log),
        () -> !intake.isDeployed()
      )
    );
  }
}
