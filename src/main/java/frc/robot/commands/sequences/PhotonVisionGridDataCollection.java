// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PhotonVisionGridDataCollection extends SequentialCommandGroup {
  
  /**
   * Drives robot in a grid, slowing down at every point, starting in the MIDDLE position, traveling between every 
   * @param x_points number of points stopped at in the x-direction; has to be ODD number
   * @param y_points number of points stopped at in the y-direction; has to be ODD number
   * @param length total distance in x-direction (forward/backward) to be traveled in meters
   * @param width total desired width in y-direction (left/right) to be traveled in meters
   * @param driveTrain
   * @param log
   */
  public PhotonVisionGridDataCollection(int x_points, int y_points, double length, double width, DriveTrain driveTrain, FileLog log) {

    Command[] commands = new Command[x_points * y_points];
    Rotation2d facingForward = new Rotation2d();

    // distance between each point in space
    double x_step = -length/(x_points - 1);
    double y_step = width/(y_points - 1);
    int idx = 0;

    for (int i = 0; i < x_points; i++) {
      if (x_points % 2 != 0) {
        for (int j = 0; j < y_points; j++) {
          commands[idx] = new DriveToPose(new Pose2d(i * x_step, j * y_step, facingForward), driveTrain, log);
          idx++;
        }
      } else {
        for (int j = y_points - 1; j >= 0; j--) {
          commands[idx] = new DriveToPose(new Pose2d(i * x_step, j * y_step, facingForward), driveTrain, log);
          idx++;
        }
      }
    }

    addCommands(
      commands
    );
  }
}