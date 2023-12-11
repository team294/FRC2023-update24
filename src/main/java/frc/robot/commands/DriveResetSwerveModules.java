// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

public class DriveResetSwerveModules extends Command {
  
  private DriveTrain driveTrain;
  private FileLog log;

  
  /**
   * Configures the motors and encoders for every swerve module.
   * The swerve modules are automatically configured in the SwerveModule constructors.  So, this
   * command should not need to be called.
   * <p>However, if the robot browns-out or otherwise partially resets, then this can be used to 
   * force the motors and encoders to have the right calibration and settings, especially the
   * calibration angle for each swerve module.
   * <p> Note:  This command can run while the robot is disabled.
   * @param driveTrain driveTrain subsytem
   * @param log FileLog
  */
  public DriveResetSwerveModules(DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    log.writeLog(false, "DriveResetSwerveModules", "Execute");
    driveTrain.configureSwerveModules();
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

  /**
   * Whether the given command should run when the robot is disabled. Override to return true if the
   * command should run when disabled.
   * @return whether the command should run when the robot is disabled
   */
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
