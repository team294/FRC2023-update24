// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;


public class DriveWithJoysticksAdvance extends Command {
  private final Joystick leftJoystick;
  private final Joystick rightJoystick;
  private final DriveTrain driveTrain;
  private final FileLog log;
  private ProfiledPIDController turnRateController;
  private boolean firstInDeadband;
  private int logRotationKey;
  private double fwdVelocity, leftVelocity, turnRate, nextTurnRate;
  private double goalAngle;       // in radians
  private double startTime;
  private boolean firstCorrecting;


    /**
   * @param leftJoystick left joystick.  X and Y axis control robot movement, relative to front of robot
   * @param rightJoystick right joystick.  X-axis controls robot rotation.
   * @param driveTrain drive train subsystem to use
   * @param log filelog to use
   */

  public DriveWithJoysticksAdvance(Joystick leftJoystick, Joystick rightJoystick, DriveTrain driveTrain, FileLog log) {
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    this.driveTrain = driveTrain;
    this.log = log;
    turnRateController = new ProfiledPIDController(DriveConstants.kPJoystickThetaController, 0, 0, TrajectoryConstants.kThetaControllerConstraints);
    turnRateController.enableContinuousInput(-Math.PI, Math.PI);


    logRotationKey = log.allocateLogRotation();

    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    driveTrain.setDriveModeCoast(false);

    goalAngle = driveTrain.getPose().getRotation().getRadians();

    firstInDeadband = true;
    firstCorrecting = true;

    turnRateController.reset(goalAngle);      // sets the current setpoint for the controller
    turnRateController.setGoal(goalAngle);    // set the goal for the controller
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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

    // SmartDashboard.putNumber("Goal Angle", goalAngle);
    // SmartDashboard.putNumber("Current Angle", driveTrain.getPose().getRotation().getRadians());

    // Uses profiled PID controller if the joystick is in the deadband
    if(turnRate == 0){
      if(firstInDeadband){
        // goalAngle = driveTrain.getPose().getRotation().getRadians();
        // goalAngle = MathUtil.angleModulus(goalAngle);
        // turnRateController.reset(goalAngle);      // sets the current setpoint for the controller
        firstInDeadband = false;
        driveTrain.enableFastLogging(true);
        startTime = System.currentTimeMillis();
      }
      if(System.currentTimeMillis() - startTime > 100){
        if(firstCorrecting){
          firstCorrecting = false;
          driveTrain.enableFastLogging(false);
          goalAngle = driveTrain.getPose().getRotation().getRadians();
          goalAngle = MathUtil.angleModulus(goalAngle);
          turnRateController.reset(goalAngle);      // sets the current setpoint for the controller
        }

      // When the right button on the right joystick is pressed then the robot turns pi radians(180 degrees)
      // This button works but it is currently used for other commands
      // if(rightJoystick.getRawButtonPressed(2)){
      //   goalAngle += Math.PI;
      //   MathUtil.angleModulus(goalAngle);
      // }

      // When the left button on the right joystick is pressed then the robot goes to 0 radians absolute
      // This button works but it is currently used for other commands
      // goalAngle = rightJoystick.getRawButtonPressed(1) ? 0 : goalAngle;

      // Calculates using the profiledPIDController what the next speed should be
        nextTurnRate = turnRateController.calculate(driveTrain.getPose().getRotation().getRadians(), goalAngle);

        if(log.isMyLogRotation(logRotationKey)) {
          log.writeLog(false, "DriveWithJoystickAdvance", "Joystick", "Fwd", fwdVelocity, "Left", leftVelocity, "Turn", nextTurnRate, "Goal Angle", goalAngle);
        }
      
        driveTrain.drive(fwdVelocity, leftVelocity, nextTurnRate, true, false);

        //firstInDeadband = false;
      }else{
        driveTrain.drive(fwdVelocity, leftVelocity, turnRate, true, false);
        
      }
    }

    // Just uses the regular turnRate if the joystick is not in the deadband
    else{
      if(log.isMyLogRotation(logRotationKey)) {
        log.writeLog(false, "DriveWithJoystickAdvance", "Joystick", "Fwd", fwdVelocity, "Left", leftVelocity, "Turn", turnRate);
      }

      driveTrain.drive(fwdVelocity, leftVelocity, turnRate, true, false);
      
      firstInDeadband = true;
      firstCorrecting = true;
    }
    
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


