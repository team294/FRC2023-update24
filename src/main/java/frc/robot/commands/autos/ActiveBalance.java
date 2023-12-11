// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ActiveBalance extends PIDCommand {
  // the setpoint for the Balance PID loop
  public static double goalAngle = 0;
  private final DriveTrain driveTrain;
  private int toleranceCounter = 0;
  
  /**
   * Actively balances the robot to the goalAngle setpoint
   * @param driveTrain driveTrain
   */
  public ActiveBalance(DriveTrain driveTrain, FileLog log) {   
    super(
        // The controller that the command will use
        new PIDController(DriveConstants.kPDriveBalance, 0, DriveConstants.kDDriveBalance),
        // This should return the measurement
        () -> driveTrain.getGyroPitch(),
        // This should return the setpoint (can also be a constant)
        () -> goalAngle,
        // This uses the output
        output -> {
          // if (Math.abs(driveTrain.getGyroPitch()) > 10) {
          //   output = -0.3*Math.signum(driveTrain.getGyroPitch());
          //   driveTrain.drive(output, 0, 0, true, false);
          // } else 
          if (Math.abs(driveTrain.getGyroPitch()) > 1) {
            driveTrain.drive(output, 0, 0, true, false);
          } else {
            driveTrain.drive(0, 0, 0, true, false);
          }
          // Use the output here
          log.writeLog(false, "ActiveBalance", "PID Loop", "Pitch", driveTrain.getGyroPitch(), "Goal Angle", goalAngle, "Output", output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(driveTrain.getGyroPitch()-goalAngle) <= 1.0) {
      toleranceCounter++;
    } else {
      toleranceCounter = 0;
    }

    return toleranceCounter>=10;
  }
}
