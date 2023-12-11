// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.*;
import frc.robot.commands.sequences.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class AutoScoreConeHigh extends SequentialCommandGroup {

  /**
   * Raise elevator to high position, score cone, and lower elevator
   */
  public AutoScoreConeHigh(boolean retract, Elevator elevator, Wrist wrist, Manipulator manipulator, Intake intake, LED led, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new FileLogWrite(true, false, "AutoScoreConeHigh", "Start", log),
      new ManipulatorSetPistonPosition(true, led, manipulator, log),		// set to cone position
      new ManipulatorSetPercent(ManipulatorConstants.pieceGrabPct, manipulator, log),				// High power to choke up on the piece
      new ElevatorWristMoveToUpperPosition(ElevatorPosition.scoreHighCone.value-2, WristAngle.upperLimit.value, elevator, wrist, intake, log),
      // new WaitCommand(0.25),
      new EjectPiece(1.0, 0.5, manipulator, log), 		// Runs for 0.5 second
      new ConditionalCommand(
        new ElevatorWristStow(elevator, wrist, log),
        new WaitCommand(0.001), 
        () -> retract
      ),
      new FileLogWrite(true, false, "AutoScoreConeHigh", "Finish", log)
    );
  }
}
