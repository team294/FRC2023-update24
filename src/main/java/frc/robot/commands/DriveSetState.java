// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

public class DriveSetState extends Command {

  private DriveTrain driveTrain;
  private FileLog log;
  private SwerveModuleState[] desiredStates;
  private boolean isOpenLoop;

  /**
   * Sets the state of all 4 swerve motors
   * @param driveSpeed speed for drive motors, in meters/sec (+ = forward)
   * @param angle angle for wheel facing, in degrees (0 = toward front of robot, + = counter clockwise, - = clockwise)
   * @param driveTrain
   * @param log
   */
  public DriveSetState(double driveSpeed, double angle, boolean isOpenLoop, DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    this.isOpenLoop = isOpenLoop;
    SwerveModuleState state = new SwerveModuleState(driveSpeed, Rotation2d.fromDegrees(angle));
    desiredStates = new SwerveModuleState[] { state, state, state, state };

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.enableFastLogging(true);
    log.writeLog(false, "DriveSetState", "Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calibrate closed-loop control in SwerveModule.setDesiredState, then decide when calling
    // this command to use true vs false.  What should the default be for this command? 
    driveTrain.setModuleStates(desiredStates, isOpenLoop);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.enableFastLogging(false);
    log.writeLog(false, "DriveSetState", "End");

    driveTrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
