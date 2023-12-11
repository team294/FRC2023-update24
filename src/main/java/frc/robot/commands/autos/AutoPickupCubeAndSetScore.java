// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.*;
import frc.robot.commands.sequences.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class AutoPickupCubeAndSetScore extends SequentialCommandGroup {

    /**
     * Drives to a cube, picks it up, drives to a midpoint, drives the robot to a scoring position,
     * elevator/wrist moves to score, ejecting the cube, and stowing elevator/wrist
     * This sequence takes care of all elevator, wrist, intake, and manipulator movement
     * needed to pick up the cube and stow the intake after getting the cube.
     * @param posCube field pose to drive to in order to pick up the cube
     * @param posNext next field pose to drive to after picking up the cube
     * @param scorePose  Scoring position on field
     * @param setScoreLowAtEnd true = move elevator/wrist to scoreLow position when moving to posNext.  false = end command with elevator/wrist still stowed.
     * @param intake
     * @param elevator
     * @param wrist
     * @param manipulator
     * @param driveTrain
     * @param led
     * @param log
     */
    public AutoPickupCubeAndSetScore (Pose2d posCube, Pose2d posNext, Pose2d scorePose, boolean setScoreLowAtEnd, 
        Intake intake, Elevator elevator, Wrist wrist, Manipulator manipulator, DriveTrain driveTrain, LED led, FileLog log) {

        addCommands(
            new FileLogWrite(true, false, "AutoPickupCubeAndSetScore", "Start", log),

            new ParallelDeadlineGroup(

                new SequentialCommandGroup(
                    // Drive to get cube
                    new DriveToPose(posCube, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kMaxRetractingAccelerationMetersPerSecondSquare,
                        TrajectoryConstants.interimPositionErrorMeters, TrajectoryConstants.interimThetaErrorDegrees, driveTrain, log),
                    // Drive to next position
                    new DriveToPose(posNext, SwerveConstants.kFullSpeedMetersPerSecond, SwerveConstants.kFullAccelerationMetersPerSecondSquare,
                        TrajectoryConstants.interimPositionErrorMeters, TrajectoryConstants.interimThetaErrorDegrees, driveTrain, log),
                    new DriveToPose(scorePose, driveTrain, log)
                    
                ),

                new SequentialCommandGroup(
                    new ElevatorWristStow(elevator, wrist, log),

                    // This next sequence sets the manipulator in cube mode, extends the intake,
                    // waits until is sees a cube, then retracts the intake and puts the manipulator in "hold cube speed"
                    new IntakeExtendAndTurnOnMotors(manipulator, intake, wrist, elevator, led, log),

                    new ConditionalCommand(
                        new ElevatorWristMoveToUpperPosition(ElevatorPosition.scoreLow.value, WristAngle.upperLimit.value, elevator, wrist, intake, log),
                        new WaitCommand(0.01),
                        () -> setScoreLowAtEnd
                    )
                )

            ),

            // Stop and retract the intake and slow down manipulator, in case the manipulator never saw a cube
            new IntakePistonSetPosition(false, intake, elevator, log),
            new IntakeStop(intake, log),
            new ManipulatorSetPistonPosition(false, led, manipulator, log),		// set to cube position
            new ManipulatorSetPercent(ManipulatorConstants.pieceHoldPct, manipulator, log),				// Low power to hold piece
            new ElevatorWristMoveToUpperPosition(ElevatorPosition.scoreLow.value, WristAngle.upperLimit.value, elevator, wrist, intake, log),


            // If we have a cube in the manipulator, then score high
            new ConditionalCommand(
                new ElevatorWristMoveToUpperPosition(ElevatorPosition.scoreHighCone.value, WristAngle.scoreMidHigh.value, elevator, wrist, intake, log),
                new WaitCommand(0.01), 
                () -> manipulator.isCubePresent()
            ),

            new FileLogWrite(true, false, "AutoPickupCubeAndSetScore", "Finish", log)
        );
    }
}