// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;
import frc.robot.utilities.FileLog;

public class ManipulatorStopMotor extends Command {
  /** Creates a new ManipulatorStopMotor. */
  private final Manipulator manipulator;
  private final FileLog log;

  public ManipulatorStopMotor(Manipulator manipulator, FileLog log) {
    this.manipulator = manipulator;
    this.log = log;

    addRequirements(manipulator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    manipulator.stopMotor();
    log.writeLog(false, "ManipulatorStopMotor", "Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
