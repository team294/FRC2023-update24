// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

public class DriveDownChargingStation extends Command {
  /** Creates a new DriveDownLoadingStation. */
  DriveTrain driveTrain;
  FileLog log;
  double speed;
  double minDistance;
  double flatPosition;
  boolean flat = false;
  public DriveDownChargingStation(double speed, double minDistance, DriveTrain driveTrain, FileLog log) {
    this.speed = speed;
    this.driveTrain = driveTrain;
    this.log = log;
    this.minDistance = minDistance;

    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flat = false;
    
    log.writeLog(false, "DriveDownCharging Station", "Initialize", "Speed", speed, "MinDistance", minDistance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.drive(speed, 0.0, 0.0, true, false);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(driveTrain.getGyroPitch()) > 4){
      flat = false;
    }
    else if(!flat){
      flatPosition = driveTrain.getPose().getX();
      flat = true;
    }
    return flat && Math.abs(driveTrain.getPose().getX() - flatPosition) > minDistance;
  }
}
