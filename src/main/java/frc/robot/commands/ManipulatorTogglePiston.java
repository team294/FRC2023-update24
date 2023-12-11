// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;
import frc.robot.utilities.FileLog;

public class ManipulatorTogglePiston extends Command {
  /** Creates a new ManipulatorStopMotor. */
  private final Manipulator manipulator;
  private final LED led;
  private final FileLog log;

  public ManipulatorTogglePiston(Manipulator manipulator, LED led, FileLog log) {
    this.manipulator = manipulator;
    this.log = log;
    this.led = led;
    addRequirements(manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    manipulator.togglePiston();
    if(manipulator.getPistonCone()){
      led.setStrip(Color.kYellow, 0);
    } else {
      led.setStrip(Color.kPurple, 0);
    }
    log.writeLog(false, "ManipulatorTogglePiston", "Initialize", "isConeAfterToggle", manipulator.getPistonCone());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
