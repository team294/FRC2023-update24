/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.MathBCR;

public class DriveResetPose extends Command {
  /**
   * Resets the pose, gyro, and encoders on the drive train
   */

  private final DriveTrain driveTrain;
  private final FileLog log;
  private final boolean fromShuffleboard;
  private final boolean onlyAngle;      // true = resent angle but not X-Y position
  private final boolean tolerance;      // true = Don't reset if within 0.5m or 15 degrees of location, false = always reset
  private double curX, curY, curAngle;    // in meters and degrees
  
  /**
	 * Resets the pose, gyro, and encoders on the drive train
   * <p> Note:  This command can run while the robot is disabled.
   * @param curXinMeters Robot X location in the field, in meters (0 = field edge in front of driver station, +=away from our drivestation)
   * @param curYinMeters Robot Y location in the field, in meters (0 = right edge of field when standing in driver station, +=left when looking from our drivestation)
   * @param curAngleinDegrees Robot angle on the field, in degrees (0 = facing away from our drivestation, + to the left, - to the right)
   * @param tolerance true = Don't reset if within 0.5m or 15 degrees of location, false = always reset
   * @param driveTrain DriveTrain subsytem
   * @param log FileLog
	 */
  public DriveResetPose(double curXinMeters, double curYinMeters, double curAngleinDegrees, boolean tolerance, DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    curX = curXinMeters;
    curY = curYinMeters;
    curAngle = curAngleinDegrees;
    fromShuffleboard = false;
    onlyAngle = false;
    this.tolerance = tolerance;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  /**
	 * Resets the pose, gyro, and encoders on the drive train
   * <p> Note:  This command can run while the robot is disabled.
   * @param curPose Robot current pose on the field.  Pose components include
   *    <p> Robot X location in the field, in meters (0 = field edge in front of driver station, +=away from our drivestation)
   *    <p> Robot Y location in the field, in meters (0 = right edge of field when standing in driver station, +=left when looking from our drivestation)
   *    <p> Robot angle on the field (0 = facing away from our drivestation, + to the left, - to the right)
   * @param tolerance true = Don't reset if within 0.5m or 15 degrees of location, false = always reset
   * @param driveTrain DriveTrain subsytem
   * @param log FileLog
	 */
  public DriveResetPose(Pose2d curPose, boolean tolerance, DriveTrain driveTrain, FileLog log) {
    this(curPose.getX(), curPose.getY(), curPose.getRotation().getDegrees(), tolerance,
        driveTrain, log);
  }

  /**
	 * Resets the pose, gyro, and encoders on the drive train.
   * Reset the angle but keep the current position (use the current measured position as the new position).
   * <p> Note:  This command can run while the robot is disabled.
   * @param curAngleinDegrees Robot angle on the field, in degrees (0 = facing away from our drivestation)
   * @param tolerance true = Don't reset if within 0.5m or 15 degrees of location, false = always reset
   * @param driveTrain DriveTrain subsytem
   * @param log FileLog
	 */
  public DriveResetPose(double curAngleinDegrees, boolean tolerance, DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    curAngle = curAngleinDegrees;
    fromShuffleboard = false;
    onlyAngle = true;
    this.tolerance = tolerance;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  /**
   * Resets the pose, gyro, and encoders on the drive train.  Gets values from Shuffleboard.
   * <p> Note:  This command can run while the robot is disabled.
   * @param driveTrain DriveTrain subystem
   * @param log FileLog
   */
  public DriveResetPose(DriveTrain driveTrain, FileLog log){
    this.driveTrain = driveTrain;
    this.log = log;
    fromShuffleboard = true;
    onlyAngle = false;
    tolerance = false;

    addRequirements(driveTrain);
    
    if(SmartDashboard.getNumber("DriveResetPose X", -9999) == -9999) {
      SmartDashboard.putNumber("DriveResetPose X", 0);
    }
    if(SmartDashboard.getNumber("DriveResetPose Y", -9999) == -9999) {
      SmartDashboard.putNumber("DriveResetPose Y", 0);
    }
    if(SmartDashboard.getNumber("DriveResetPose Angle", -9999) == -9999){
      SmartDashboard.putNumber("DriveResetPose Angle", 0);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(fromShuffleboard){
      curX = SmartDashboard.getNumber("DriveResetPose X", 0);
      curY = SmartDashboard.getNumber("DriveResetPose Y", 0);
      curAngle = SmartDashboard.getNumber("DriveResetPose Angle", 0);
    }

    if(onlyAngle){
      curX = driveTrain.getPose().getX();
      curY = driveTrain.getPose().getY();
    }
    
    log.writeLog(false, "DriveResetPose", "Init", "X", curX, "Y", curY, "Angle", curAngle);

    if( !tolerance ||
        Math.abs(curX - driveTrain.getPose().getX()) > 0.5 || 
        Math.abs(curY - driveTrain.getPose().getY()) > 0.5 || 
        Math.abs(MathBCR.normalizeAngle(curAngle - driveTrain.getPose().getRotation().getDegrees())) > 15.0 ) {
      driveTrain.resetPose(new Pose2d(curX, curY, Rotation2d.fromDegrees(curAngle)));
    }
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
