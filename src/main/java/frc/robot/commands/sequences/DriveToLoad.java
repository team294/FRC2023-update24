package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.*;
import frc.robot.commands.ManipulatorGrab.BehaviorType;
import frc.robot.subsystems.*;
import frc.robot.utilities.Field;
import frc.robot.utilities.FileLog;

public class DriveToLoad extends SequentialCommandGroup {

  /**
   * Drives robot to loading position in front of loading station, 
   * extends elevator, turns on manipulator, drives forward, and
   * grabs game piece
   * @param driveTrain
   * @param wrist
   * @param elevator
   * @param manipulator
   * @param intake
   * @param field
   * @param log
   */
  public DriveToLoad(DriveTrain driveTrain, Wrist wrist, Elevator elevator, Manipulator manipulator, Intake intake, Field field, FileLog log) {
    addCommands(
      new DriveToPose(field.getLoadingPositionInitial(),
        SwerveConstants.kFullSpeedMetersPerSecond, SwerveConstants.kFullAccelerationMetersPerSecondSquare,
        TrajectoryConstants.interimPositionErrorMeters, TrajectoryConstants.interimThetaErrorDegrees, driveTrain, log),
      new ManipulatorGrab(ManipulatorConstants.pieceGrabPct, BehaviorType.immediatelyEnd, manipulator, log),
      new ConditionalCommand(
        new ElevatorWristMoveToUpperPosition(ElevatorPosition.loadingStationCone.value, WristAngle.loadHumanStation.value, elevator, wrist, intake, log), 
        new ElevatorWristMoveToUpperPosition(ElevatorPosition.loadingStationCube.value, WristAngle.loadHumanStation.value, elevator, wrist, intake, log), 
        manipulator::getPistonCone
      ),
      // new ManipulatorGrab(0.8, BehaviorType.immediatelyEnd, manipulator, log),
      // new DriveToPose(field.getLoadingPositionFinal(), driveTrain, log).until(() -> (manipulator.isConePresent() || manipulator.isCubePresent())),
      new ManipulatorGrab(ManipulatorConstants.pieceGrabPct, BehaviorType.waitForConeOrCube, manipulator, log),
      new DriveStop(driveTrain, log)
      // new ManipulatorGrab(0.8, BehaviorType.waitForConeOrCube, manipulator, log)
      );
  }
}
