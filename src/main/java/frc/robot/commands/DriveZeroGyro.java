/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

public class DriveZeroGyro extends Command {
  /**
   * Zeros gyro on the drive train
   */

  private DriveTrain driveTrain;
  private FileLog log;
  private double zeroAngle;

  /**
	 * Zero the gyro position in software.
   * <p> Note:  This command can run while the robot is disabled.
   * @param driveTrain DriveTrain subsytem
   * @param log FileLog
	 */
  public DriveZeroGyro(DriveTrain driveTrain, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.log = log;
    zeroAngle = 0;
    addRequirements(driveTrain);
  }

  /**
	 * Zero the gyro position in software against a specified angle.
   * <p> Note:  This command can run while the robot is disabled.
	 * @param zeroAngle current robot angle compared to the zero angle
   * @param driveTrain DriveTrain subsytem
   * @param log2 FileLog
	 */
  public DriveZeroGyro(double zeroAngle, DriveTrain driveTrain, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.log = log;
    this.zeroAngle = zeroAngle;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.zeroGyroRotation(zeroAngle);
    log.writeLog(false, "DriveZeroGyro", "Init", "CurrAng", zeroAngle);
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
