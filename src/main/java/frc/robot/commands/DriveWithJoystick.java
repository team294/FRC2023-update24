/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.*;

/**
 * Command to control the drive train with joysticks using arcade drive.
 */
public class DriveWithJoystick extends Command {
  private final DriveTrain driveTrain;
  private final Joystick leftJoystick;
  private final Joystick rightJoystick;
  private final FileLog log;
  private int logRotationKey;
  
  private double fwdVelocity, leftVelocity, turnRate;
  // private double lastFwdPercent, lastTime, curTime;

  // private final double maxFwdRateChange = 2.0;
  // private final double maxRevRateChange = -1.4;

  /**
   * @param leftJoystick left joystick.  X and Y axis control robot movement, relative to front of robot
   * @param rightJoystick right joystick.  X-axis controls robot rotation.
   * @param driveTrain drive train subsystem to use
   * @param log filelog to use
   */
  public DriveWithJoystick(Joystick leftJoystick, Joystick rightJoystick, DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    logRotationKey = log.allocateLogRotation();

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setDriveModeCoast(false);

    // lastFwdPercent = 0;
    // lastTime = System.currentTimeMillis() / 1000.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // curTime = System.currentTimeMillis() / 1000.0;
    fwdVelocity = -leftJoystick.getY();
    leftVelocity = -leftJoystick.getX();
    turnRate = -rightJoystick.getX();
    SmartDashboard.putNumber("Left Joystick Y", fwdVelocity);
    SmartDashboard.putNumber("Left Joystick X", leftVelocity);
    SmartDashboard.putNumber("Right Joystick X", turnRate);


    // Apply deadbands
    fwdVelocity = (Math.abs(fwdVelocity) < OIConstants.joystickDeadband) ? 0 : scaleJoystick(fwdVelocity) * SwerveConstants.kMaxSpeedMetersPerSecond;
    leftVelocity = (Math.abs(leftVelocity) < OIConstants.joystickDeadband) ? 0 : scaleJoystick(leftVelocity) * SwerveConstants.kMaxSpeedMetersPerSecond;
    turnRate = (Math.abs(turnRate) < OIConstants.joystickDeadband) ? 0 : scaleTurn(turnRate) * SwerveConstants.kMaxTurningRadiansPerSecond;

    if(log.isMyLogRotation(logRotationKey)) {
      log.writeLog(false, "DriveWithJoystickArcade", "Joystick", "Fwd", fwdVelocity, "Left", leftVelocity, "Turn", turnRate);
    }
    
    // double fwdRateChange = (fwdPercent - lastFwdPercent) / (curTime - lastTime);
    // if (fwdRateChange > maxFwdRateChange) {
    //   fwdPercent = lastFwdPercent + (curTime - lastTime)*maxFwdRateChange;
    // } else if (fwdRateChange < maxRevRateChange) {
    //   fwdPercent = lastFwdPercent +(curTime - lastTime)*maxRevRateChange;

    // }
    
    driveTrain.drive(fwdVelocity, leftVelocity, turnRate, true, false);

    // lastFwdPercent = fwdPercent;
    // lastTime = curTime;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Re-maps joystick value to better enable fine robot control at small joystick
   * values (low speeds) and full-speed travel at large joystick values.
   * This method is optimized for linear travel.
   * @param rawJoystick Raw joystick value, -1.0 to +1.0
   * @return Scaled joystick value, -1.0 to +1.0
   */
  private double scaleTurn(double rawJoystick){
    return Math.signum(rawJoystick)*(0.6801 * rawJoystick * rawJoystick + 0.3232 * Math.abs(rawJoystick) - 0.0033);
  }

  /**
   * Re-maps joystick value to better enable fine robot control at small joystick
   * values (low speeds) and full-speed travel at large joystick values.
   * This method is optimized for rotating the robot.
   * @param rawJoystick Raw joystick value, -1.0 to +1.0
   * @return Scaled joystick value, -1.0 to +1.0
   */
  private double scaleJoystick(double rawJoystick){
    return Math.signum(rawJoystick)*(0.7912*rawJoystick*rawJoystick + 0.2109*Math.abs(rawJoystick) - 0.0022);
  }
}
