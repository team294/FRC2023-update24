// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

public class ActiveBalanceTwo extends Command {
  /** Creates a new ActiveBalanceTwo. */
  DriveTrain driveTrain;
  FileLog log;
  public ActiveBalanceTwo(DriveTrain driveTrain, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.log = log;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "ActiveBalanceTwo", "init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(driveTrain.getGyroPitch() > 9){
      driveTrain.drive(-TrajectoryConstants.ChargeStationBalanceVelocity, 0, 0, true, false);
    }else if(driveTrain.getGyroPitch() < -9){
      driveTrain.drive(TrajectoryConstants.ChargeStationBalanceVelocity, 0, 0, true, false);
    }else{
      driveTrain.drive(0, 0, 0.01, true, false);// Stop motor and Lock the wheels at 45deg
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
