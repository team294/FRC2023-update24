// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class ManipulatorGrab extends Command {

    public enum BehaviorType{
      immediatelyEnd,
      runForever, 
      waitForCone,
      waitForCube,
      waitForConeOrCube
    }

    private final Manipulator manipulator;
    private final FileLog log;
    private final Timer timer = new Timer();

    private double motorPercent = 0.0;
    // private double ampSensitivity = 0.0;
    // * @param ampSensitivity [Double] minimum value sensitivity must change by to detect that manipulator has picked up object
   
    private BehaviorType behaviorType;

    

    /**
   * Command with 5 different behaviors for the manipulator (cone-cube grabber)
   *immdiatelyEnd = immediately end command, motor stays at set speed
    runForever = command stays running forever, stop when interrupted
    waitForCone = wait for cone, then stop motor
    waitForCube = wait for cube, then stop motor
    waitForConeOrCube = wait for cone or cube then stop motor
   * @param MotorPercent [Double] percent motors will be run at when command is initialized.   -1 (eject) to +1 (grab)
   * @param BehaviorType [BehaviorType] enum that defines behavior of manipulator
   * @param Manipulator [Manipulator] manipulator object
   * @param log
   */
  public ManipulatorGrab(double motorPercent, BehaviorType behaviorType,  Manipulator manipulator, FileLog log) {
    this.manipulator = manipulator;
    this.log = log;
    this.motorPercent = motorPercent;
    this.behaviorType = behaviorType;
    // this.ampSensitivity = ampSensitivity;

    addRequirements(manipulator);
  }

  // Called when the command is initially scheduled.
 
  /*sets motor percent to parameter passed in constructor
   if behavior is set to coneGrab then toggle manipulator to grab cones, vice versa for cubes
   */

   @Override
  public void initialize() {
    manipulator.setMotorPercentOutput(motorPercent);

    if(behaviorType == BehaviorType.waitForCone){
      manipulator.setPistonCone(true);

    } else if(behaviorType == BehaviorType.waitForCube){
      manipulator.setPistonCone(false);

    }
    log.writeLog(false, "ManipulatorGet", "Start", "Percent", motorPercent, "Behavior", behaviorType);

    timer.stop();
    timer.reset();
  }

  //Called in a loop every time the command is executed

  //checks for amp spike greater than ampSensitivity parameter, toggles coneGrab and cubeGrab booleans depending on the configuration of the manipulator
  @Override
  public void execute(){
    log.writeLog(false, "ManipulatorGet", "Amps", manipulator.getAmps(), "Motor Percent Output", motorPercent);
  }

  /*end condition behaviors for finished command
    
    -immediately end command, motor stays at set speed
    -command stays running forever, stop when interrupted
    -wait for cone, then stop motor
    -wait for cube, then stop motor
    -wait for cone or cube then stop motor
   */
  @Override
  public boolean isFinished(){
    //boolean notNull = grabType != null;
    //using amps
    // boolean hasObject = manipulator.getAmps() > ampSensitivity;
    //using sensor
    switch (behaviorType) {
      case immediatelyEnd:
        return true;  
      case runForever:
        return false;
      case waitForCone:
        if (manipulator.isConePresent()) {
          timer.start();
        }
        return (manipulator.isConePresent() && timer.get()>=0.3) ;
      case waitForCube:
        if (manipulator.isCubePresent()) {
          timer.start();
        }
        return (manipulator.isCubePresent() && timer.get()>=0.3) ;
      case waitForConeOrCube:
        if(manipulator.getPistonCone()){
          if (manipulator.isConePresent()) {
            timer.start();
          }
          return (manipulator.isConePresent() && timer.get()>=0.3) ;
        }
        if (manipulator.isCubePresent()) {
          timer.start();
        }
        return (manipulator.isCubePresent() && timer.get()>=0.3) ;
  
      default:
        return true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (behaviorType == BehaviorType.waitForCone || behaviorType == BehaviorType.waitForCube ||
        behaviorType == BehaviorType.waitForConeOrCube) {
      manipulator.setMotorPercentOutput(ManipulatorConstants.pieceHoldPct);
    } else if(behaviorType == BehaviorType.immediatelyEnd){
      manipulator.setMotorPercentOutput(motorPercent);
    }else {
      manipulator.stopMotor();
    }
  }

}
