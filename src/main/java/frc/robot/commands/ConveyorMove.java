// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyor;
import frc.robot.utilities.FileLog;

public class ConveyorMove extends Command {
  /** Creates a new conveyorMove. */
  private final Conveyor conveyor;
  private final FileLog log;
  //percentage speed of the motor
  private double speed = 0;
  private boolean fromShuffleboard = false;

  public ConveyorMove(double speed, Conveyor conveyor, FileLog log) {
    this.conveyor = conveyor;
    this.log = log;
    fromShuffleboard = false;
    this.speed = speed;
    addRequirements(conveyor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ConveyorMove(Conveyor conveyor, FileLog log) {
    this.conveyor = conveyor;
    this.log = log;
    fromShuffleboard = true;
    addRequirements(conveyor);
    if(SmartDashboard.getNumber("Conveyor Percent", -9999) == -9999) {
        SmartDashboard.putNumber("Conveyor Percent", 0);
    }
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(fromShuffleboard){
        speed = SmartDashboard.getNumber("Conveyor Percent",0);
    }
    conveyor.setMotorPercentOutput(speed);
    log.writeLog(false, conveyor.getName(), "conveyor activated");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // conveyor.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
