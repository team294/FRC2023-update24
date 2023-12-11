// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class IntakeRetractAndTurnOffMotors extends SequentialCommandGroup {

  /**
   * Retracts intake, waits one second, turns off intake motor
   * @param intake
   * @param elevator
   * @param log
   */
  public IntakeRetractAndTurnOffMotors(Intake intake, Elevator elevator, FileLog log) {
    addCommands(
      new IntakePistonSetPosition(false, intake, elevator, log),
      new WaitCommand(1),
      new IntakeStop(intake, log)
    );
  }
}
