// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class EjectPiece extends SequentialCommandGroup {


  /**
   * Runs the manipulator motor to eject a piece for 1 second then stops the motor
   * @param manipulator
   * @param log
   */
  public EjectPiece(Manipulator manipulator, FileLog log) {
    addCommands(
      new ManipulatorSetPercent(-0.5, manipulator, log),
      new WaitCommand(1.0),
      new ManipulatorStopMotor(manipulator, log)
    );
  }
  
  /**
   * Runs the manipulator motor to eject a piece at percent speed for delay seconds then stops the motor
   * @param percent percent to run motor at (positive value ejects piece)
   * @param delay delay (in seconds)
   * @param manipulator
   * @param log
   */
  public EjectPiece(double percent, double delay, Manipulator manipulator, FileLog log) {
    addCommands(
      new ManipulatorSetPercent(-percent, manipulator, log),
      new WaitCommand(delay),
      new ManipulatorStopMotor(manipulator, log)
    );
  }
}
