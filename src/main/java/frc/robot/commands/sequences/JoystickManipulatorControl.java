// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.ManipulatorGrab;
import frc.robot.commands.ManipulatorGrab.BehaviorType;
import frc.robot.subsystems.Manipulator;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class JoystickManipulatorControl extends SequentialCommandGroup {
  /** Creates a new JoystickManipulatorControl. */
  public JoystickManipulatorControl(Joystick joystick, Manipulator manipulator, FileLog log) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ConditionalCommand(
        new ManipulatorGrab(ManipulatorConstants.pieceGrabPct, BehaviorType.runForever, manipulator, log), 
        new WaitCommand(0), 
        () -> joystick.getY() > .95
      ),
      new ConditionalCommand(
        new EjectPiece(manipulator, log),
        new WaitCommand(0), 
        () -> joystick.getY() < -.95
      )
      


    );
  }
}
