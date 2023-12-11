// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.ManipulatorGrab.BehaviorType;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class IntakeExtendAndTurnOnMotors extends SequentialCommandGroup {
  /**
   * Extends intake, sets manipulator to cube mode, and runs intake and manipulator motors.
   * Once a piece is detected in the manipulator, retracts intake.
   * Only runs if the elevator is down.  If the elevator is not down, then does nothing.
   * @param manipulator
   * @param intake
   * @param elevator
   * @param log
   */
  public IntakeExtendAndTurnOnMotors(Manipulator manipulator, Intake intake, Wrist wrist, Elevator elevator, LED led, FileLog log) {
    addCommands(
      new FileLogWrite(false, false, "IntakeExtendAndTurnOnMotors", "Start", log),
      new ConditionalCommand(
        new SequentialCommandGroup(
          new IntakePistonSetPosition(true, intake, elevator, log),
          new IntakeSetPercentOutput(IntakeConstants.motor1NominalIntakePct, IntakeConstants.motor2NominalIntakePct, intake, log),
          new ManipulatorSetPistonPosition(false, led, manipulator, log),     // Set manipulator to cube
          new ElevatorWristStow(elevator, wrist, log),

          // Grab piece, wait for piece to be held
          new ManipulatorGrab(ManipulatorConstants.pieceGrabFromIntakePct, BehaviorType.waitForConeOrCube, manipulator, log),

          // Retract and turn off intake
          // new IntakeRetractAndTurnOffMotors(intake, elevator, log),
          new IntakePistonSetPosition(false, intake, elevator, log),
          new IntakeStop(intake, log),
    
          new FileLogWrite(false, false, "IntakeExtendAndTurnOnMotors", "End", log)
        ),
        new FileLogWrite(false, false, "IntakeExtendAndTurnOnMotors", "Elevator Not Down", log),
        () -> (elevator.getElevatorPos() <= ElevatorConstants.boundBottomMain)
      )
    );
  }
}
