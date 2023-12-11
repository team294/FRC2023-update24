// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.FileLog;

public class IntakeSetPercentOutput extends Command {
  /** Creates a new IntakeSetPercentOutput. */
  private Intake intake;
  private double intake1PercentOutput;
  private double intake2PercentOutput;
  private FileLog log;

  /**
   * Sets the percent output of the intake motor, + is intake, - is outtake.
   * This commmand ends immediately with the intake still running.
   * @param intake1PercentOutput -1.0 to +1.0, Motor that deploys
   * @param intake2PercentOutput -1.0 to +1.0, Motor that doesn't deploy
   * @param intake intake subsystem
   * @param log log file
   */
  public IntakeSetPercentOutput(double intake1PercentOutput, double intake2PercentOutput, Intake intake, FileLog log) {
    this.intake = intake;
    this.intake1PercentOutput = intake1PercentOutput;
    this.intake2PercentOutput = intake2PercentOutput;
    this.log = log;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "IntakeSetPercentOutput", "Initialize" ,"Motor 1 Intake Percent", intake1PercentOutput, "Motor 2 Intake Percent", intake2PercentOutput);
    intake.setMotorPercentOutput(intake1PercentOutput, intake2PercentOutput);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
