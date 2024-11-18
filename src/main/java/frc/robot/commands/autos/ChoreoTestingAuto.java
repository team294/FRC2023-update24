package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.FileLogWrite;
import frc.robot.commands.ManipulatorSetPercent;
import frc.robot.commands.ManipulatorSetPistonPosition;
import frc.robot.commands.sequences.EjectPiece;
import frc.robot.commands.sequences.ElevatorWristMoveToUpperPosition;
import frc.robot.commands.sequences.ElevatorWristStow;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;

public class ChoreoTestingAuto extends SequentialCommandGroup {
    public ChoreoTestingAuto(DriveTrain driveTrain, LED led, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
      new FileLogWrite(true, false, "ChoreoTestingTrajectory", "Start", log)
    );
  }
}
