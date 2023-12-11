// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.*;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.*;
import frc.robot.commands.sequences.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class AutoScoreCube extends SequentialCommandGroup {

  /**
   * Scores a cube, including driving the robot to the scoring position, elevator/wrist moves to score,
   * ejecting the cube, and stowing the elevator/wrist.
   * @param scorePose  Scoring position on field
   * @param driveTrain
   * @param elevator
   * @param wrist
   * @param manipulator
   * @param intake
   * @param led
   * @param log
   */
  public AutoScoreCube(Pose2d scorePose,
      DriveTrain driveTrain, Elevator elevator, Wrist wrist, 
      Manipulator manipulator, Intake intake, LED led, FileLog log) {
    addCommands(
      new FileLogWrite(true, false, "AutoScoreCube", "Start", log),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new ManipulatorSetPistonPosition(false, led, manipulator, log),		// set to cube position
          new ManipulatorSetPercent(ManipulatorConstants.pieceHoldPct, manipulator, log),				// Low power to hold piece
          new ElevatorWristMoveToUpperPosition(ElevatorPosition.scoreLow.value, WristAngle.upperLimit.value, elevator, wrist, intake, log)
        ),
        new DriveToPose(scorePose, driveTrain, log)
      ),

      // If we have a cube in the manipulator, then score high
      new ConditionalCommand(
        new ElevatorWristMoveToUpperPosition(ElevatorPosition.scoreHighCone.value, WristAngle.scoreMidHigh.value, elevator, wrist, intake, log),
        // new ElevatorWristMoveToUpperPosition(ElevatorPosition.scoreLow.value, WristAngle.scoreMidHigh.value, elevator, wrist, intake, log),
        new WaitCommand(0.01), 
        () -> manipulator.isCubePresent()
      ),

      new EjectPiece(manipulator, log), 		// Runs for 1 second
      new ElevatorWristStow(elevator, wrist, log),
      new FileLogWrite(true, false, "AutoScoreCube", "Finish", log)
    );
  }
}
