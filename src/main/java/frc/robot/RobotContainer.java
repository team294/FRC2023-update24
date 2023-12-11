// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.StopType;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.*;
import frc.robot.commands.ManipulatorGrab.BehaviorType;
import frc.robot.commands.autos.*;
import frc.robot.commands.sequences.*;
import frc.robot.subsystems.*;
// import frc.robot.triggers.*;
import frc.robot.utilities.*;
import frc.robot.utilities.TrajectoryCache.TrajectoryFacing;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Define robot key utilities (DO THIS FIRST)
  private final FileLog log = new FileLog("H5");
  private final AllianceSelection allianceSelection = new AllianceSelection(log);
  private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  private final Field field = new Field(allianceSelection, log);

  // Define robot subsystems  
  private final Wrist wrist = new Wrist(log);
  private final Elevator elevator = new Elevator(wrist, log);
  private final DriveTrain driveTrain = new DriveTrain(field, elevator, log);
  private final Manipulator manipulator = new Manipulator(log);
  private final Intake intake = new Intake(log);
  private final LED led = new LED();
  // private final Conveyor conveyor = new Conveyor(log);

  // Define other utilities
  private final TrajectoryCache trajectoryCache = new TrajectoryCache(log);
  private final AutoSelection autoSelection = new AutoSelection(trajectoryCache, allianceSelection, field, log);

  // Define controllers
  // private final Joystick xboxController = new Joystick(OIConstants.usbXboxController); //assuming usbxboxcontroller is int
  private final Joystick leftJoystick = new Joystick(OIConstants.usbLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.usbRightJoystick);
  private final Joystick coPanel = new Joystick(OIConstants.usbCoPanel);

  private final CommandXboxController xboxController = new CommandXboxController(OIConstants.usbXboxController);
  private boolean rumbling = false;
  private boolean lastEnabledModeAuto = false;    // True if the last mode was auto

  // Set to this pattern when the robot is disabled
  private final Command patternTeamMoving = new LEDSetPattern(LED.teamMovingColorsLibrary, 0, 60, led, log);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings(); // configure button bindings
    configureShuffleboard(); // configure shuffleboard

    // driveTrain.setDefaultCommand(new DriveWithJoystick(leftJoystick, rightJoystick, driveTrain, log));
    driveTrain.setDefaultCommand(new DriveWithJoysticksAdvance(leftJoystick, rightJoystick, driveTrain, log));

  }

    /**
   * Define Shuffleboard mappings.
   */
  private void configureShuffleboard() {

    // display sticky faults
    RobotPreferences.showStickyFaultsOnShuffleboard();
    SmartDashboard.putData("Clear Sticky Faults", new StickyFaultsClear(log));

    // Testing for drivetrain autos and trajectories
    SmartDashboard.putData("Drive Reset SwerveModules", new DriveResetSwerveModules(driveTrain, log));
    SmartDashboard.putData("Zero Gyro", new DriveZeroGyro(driveTrain, log));
    SmartDashboard.putData("Zero Odometry", new DriveResetPose(0, 0, 0, false, driveTrain, log));
    // SmartDashboard.putData("Set Odometry if out of tol", new DriveResetPose(2, 2, 180, true, driveTrain, log));      // For testing only
    SmartDashboard.putData("Drive Reset Pose", new DriveResetPose(driveTrain, log));
    SmartDashboard.putData("Calibrate Drive Motors", new DriveCalibration(0.5, 12, 0.05, driveTrain, log));
    SmartDashboard.putData("Calibrate Turn Motors", new DriveTurnCalibration(1.0, 10, 0.2, driveTrain, log));
    SmartDashboard.putData("Drive Wheels 0 deg", new DriveSetState(0, 0, false, driveTrain, log));
    SmartDashboard.putData("Drive Wheels +85 deg", new DriveSetState(0, 85, false, driveTrain, log));
    SmartDashboard.putData("Drive Wheels +95 deg", new DriveSetState(0, 95, false, driveTrain, log));
    SmartDashboard.putData("Drive 1.5 mps 0 deg", new DriveSetState(1.5, 0, false, driveTrain, log));
    SmartDashboard.putData("Drive Straight", new DriveStraight(false, false, false, driveTrain, log));
    SmartDashboard.putData("Drive Lock Wheels", new DriveToPose(CoordType.kRelative, 0.5, driveTrain, log));

    // Testing for trajectories
    Rotation2d rotationFront = new Rotation2d();          // Facing away from drivers
    SmartDashboard.putData("Drive To Pose", new DriveToPose(driveTrain, log));
    SmartDashboard.putData("Drive To Pose Test", new DriveToPose(new Pose2d(1, 1, Rotation2d.fromDegrees(0)), driveTrain, log));
    // SmartDashboard.putData("Drive To Pose Test", new Pose2d(16.17878-1.7, 6.749796-0.25, new Rotation2d(0), driveTrain, log));
    SmartDashboard.putData("Drive To Loading4", new DriveToPose(new Pose2d(16.17878-1.0, 6.749796, new Rotation2d(0)),
      SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare,
      TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, driveTrain, log));
    SmartDashboard.putData("Drive to Load Station", new DriveToLoad(driveTrain, wrist, elevator, manipulator, intake, field, log));
    // SmartDashboard.putData("Drive Trajectory Relative", new DriveTrajectory(CoordType.kRelative, StopType.kBrake, 
    //     trajectoryCache.cache[TrajectoryType.test.value], driveTrain, log));
    // SmartDashboard.putData("Drive Trajectory Curve Relative", new DriveTrajectory(CoordType.kRelative, StopType.kBrake, 
    //     trajectoryCache.cache[TrajectoryType.testCurve.value], driveTrain, log));
    // SmartDashboard.putData("Drive Trajectory Absolute", new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, 
    //     trajectoryCache.cache[TrajectoryType.test.value], driveTrain, log));  
    SmartDashboard.putData("Drive Trajectory Straight", new DriveTrajectory(
          CoordType.kRelative, StopType.kBrake,
          new TrajectoryFacing(rotationFront, rotationFront, 
            TrajectoryGenerator.generateTrajectory(
              new Pose2d(0,0,new Rotation2d(0)), 
              List.of(), 
              new Pose2d(1.0,0,new Rotation2d(0)), 
              Constants.TrajectoryConstants.swerveTrajectoryConfig
            )
          ),
          driveTrain, log));
    SmartDashboard.putData("Drive to closest goal", new DriveToPose(() -> field.getInitialColumn(field.getClosestGoal(driveTrain.getPose(), manipulator.getPistonCone())), driveTrain, log));
    // SmartDashboard.putData("Drive Smart Balance", new SequentialCommandGroup(new ResetPose,new SmartBalance(0.5, 0, driveTrain)));
    // SmartDashboard.putData("Test Balance", new SequentialCommandGroup(new DriveUpChargingStation(-TrajectoryConstants.ChargeStationVelocity, .889, driveTrain, log), new ActiveBalance(driveTrain, log)));
    SmartDashboard.putData("Test Balance", new ActiveBalance(driveTrain, log));
    // Testing for autos
    // SmartDashboard.putData("Example Auto S-Shape", new ExampleAuto(driveTrain));
    // SmartDashboard.putData("Center Balance Blue", new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, 
    //     trajectoryCache.cache[TrajectoryType.CenterBalanceBlue.value], driveTrain, log));
    // SmartDashboard.putData("Center Balance Community Blue", new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, 
    //     trajectoryCache.cache[TrajectoryType.MiddleOuterOneConeBalanceBlue.value], driveTrain, log));
    // SmartDashboard.putData("Auto OneConeBalance", new OuterOneConeBalanceMiddleAuto(driveTrain));
  
    //Elevator Commands
    SmartDashboard.putData("Elevator Cal Encoder", new ElevatorCalibrateEncoderIfAtLowerLimit(elevator, log));
    SmartDashboard.putData("Elevator Calibration", new ElevatorCalibration(1, elevator, log));
    SmartDashboard.putData("Elevator Set Percent", new ElevatorSetPercentOutput(elevator, log));
    SmartDashboard.putData("Elevator Set Position", new ElevatorSetPosition(elevator, log));
    SmartDashboard.putData("Elevator Move To Bottom", new ElevatorSetPosition(ElevatorPosition.bottom, elevator, log));
    
    //Wrist Commands
    SmartDashboard.putData("Wrist Start Position", new WristSetAngle(WristAngle.startConfig, wrist, log));
    SmartDashboard.putData("Wrist ElevMove Position", new WristSetAngle(WristAngle.elevatorMoving, wrist, log));
    SmartDashboard.putData("Wrist Scoring Position", new WristSetAngle(WristAngle.scoreMidHigh, wrist, log));
    SmartDashboard.putData("Wrist Set Angle", new WristSetAngle(wrist, log));
    SmartDashboard.putData("Wrist Set Output", new WristSetPercentOutput(wrist, log));

    //LED commands
    SmartDashboard.putData("LED Rainbow", new LEDSetPattern(LED.rainbowLibrary, 0, 0.5, led, log));
    SmartDashboard.putData("LED Flash Team Color", new LEDSetPattern(LED.teamFlashingColorsLibrary, 0, 0.5, led, log));
    SmartDashboard.putData("LED Full Team Color", new LEDSetPattern(LED.teamFullColorsLibrary, 0, 0.5, led, log));
    SmartDashboard.putData("LED moving Team Color", new LEDSetPattern(LED.teamMovingColorsLibrary, 0, 0.5, led, log));
    SmartDashboard.putData("LED OFF", new LEDSetStrip(Color.kBlack, 0, led, log));
    SmartDashboard.putData("LED Yellow", new LEDSetStrip(Color.kYellow, 1, led, log));
    SmartDashboard.putData("LED Purple", new LEDSetStrip(Color.kPurple, 1, led, log));

    //Conveyor Commands
    // SmartDashboard.putData("Conveyor Custom Percent", new ConveyorMove(conveyor, log));
    // SmartDashboard.putData("Conveyor Run", new ConveyorMove(0.3, conveyor, log));
    // SmartDashboard.putData("Conveyor Stop", new ConveyorMove(0, conveyor, log));

    // Intake Commands
    SmartDashboard.putData("Intake Stop", new IntakeStop(intake, log));
    SmartDashboard.putData("Intake Pick Up",new IntakeSetPercentOutput(.75, .35, intake, log));
    SmartDashboard.putData("Intake Eject",new IntakeSetPercentOutput(-0.5, -.25, intake, log));
    SmartDashboard.putData("Intake Deploy", new IntakePistonSetPosition(true, intake, elevator, log));
    SmartDashboard.putData("Intake Stow", new IntakePistonSetPosition(false, intake, elevator, log));

    //Manipulator Commands
    SmartDashboard.putData("Manipulator Stop", new ManipulatorStopMotor(manipulator, log));
    SmartDashboard.putData("Manipulator Pick Up",new ManipulatorGrab(ManipulatorConstants.pieceGrabPct, BehaviorType.waitForConeOrCube, manipulator, log));
    SmartDashboard.putData("Manipulator Eject",new ManipulatorSetPercent(-0.5, manipulator, log));
    SmartDashboard.putData("Manipulator Cone", new ManipulatorSetPistonPosition(true, led, manipulator, log));
    SmartDashboard.putData("Manipulator Cube", new ManipulatorSetPistonPosition(false, led, manipulator, log));
    SmartDashboard.putData("Manipulator Toggle", new ManipulatorTogglePiston(manipulator, led, log));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureXboxButtons(); // configure xbox controller
    configureJoystickButtons(); // configure joysticks
    configureCopanel(); // configure copanel
  }

  /**
   * Configures XBox buttons and controls
   */
  private void configureXboxButtons(){
    //check povtrigger and axis trigger number bindings
    
    // Triggers for all xbox buttons
  
    Trigger xbLT = xboxController.leftTrigger();
    Trigger xbRT = xboxController.rightTrigger();
    Trigger xbA = xboxController.a();
    Trigger xbB = xboxController.b();
    Trigger xbY = xboxController.y();
    Trigger xbX = xboxController.x();
    Trigger xbLB = xboxController.leftBumper();
    Trigger xbRB = xboxController.rightBumper();
    Trigger xbBack = xboxController.back();
    Trigger xbStart = xboxController.start();
    Trigger xbPOVUp = xboxController.povUp();
    Trigger xbPOVRight = xboxController.povRight();
    Trigger xbPOVLeft = xboxController.povLeft();
    Trigger xbPOVDown = xboxController.povDown();
   
    //a
    // xbA.onTrue(new ElevatorSetPosition(ElevatorPosition.scoreLow, elevator, log)); 
    // Move elevator/wrist to score low position
    xbA.onTrue(
      new ConditionalCommand(
        // Raise Elevator.  Schedule the elevator command separately, so this xbox button does not
        // require the intake susbsytem outside of the conditional command.  That will prevent
        // the conditional command from interrupting other commands if the intake is out.
        // When the intake is in, the subsystem requirements will take effect when ElevatorWristMoveToUpperPosition
        // is scheduled. 
        new ScheduleCommand(new ElevatorWristMoveToUpperPosition(ElevatorPosition.scoreLow.value, WristAngle.upperLimit.value, elevator, wrist, intake, log)),
        new FileLogWrite(false, false, "Xbox A", "Intake is out", log),
        () -> (!intake.isDeployed()) 
      ) // ONLY allow this command to run if the intake is in; otherwise would interrupt the ManipulatorGrab command
    );
    //b
    // xbB.onTrue(new ElevatorSetPosition(ElevatorPosition.scoreMidCone, elevator, log));         
    // Move elevator/wrist to score mid position
    xbB.onTrue(
      new ConditionalCommand(
        // Raise Elevator.  Schedule the elevator command separately, so this xbox button does not
        // require the intake susbsytem outside of the conditional command.  That will prevent
        // the conditional command from interrupting other commands if the intake is out.
        // When the intake is in, the subsystem requirements will take effect when ElevatorWristMoveToUpperPosition
        // is scheduled. 
        new ScheduleCommand(new ElevatorWristMoveToUpperPosition(ElevatorPosition.scoreMidCone.value, WristAngle.scoreMidHigh.value, elevator, wrist, intake, log)),
        new FileLogWrite(false, false, "Xbox B", "Intake is out", log),
        () -> (!intake.isDeployed()) 
      ) // ONLY allow this command to run if the intake is in; otherwise would interrupt the ManipulatorGrab command  
    );
 
    //y
    // xbY.onTrue(new ElevatorSetPosition(ElevatorPosition.scoreHighCone, elevator, log));
    // Move elevator/wrist to score high position
    xbY.onTrue(
      new ConditionalCommand(
        // Raise Elevator.  Schedule the elevator command separately, so this xbox button does not
        // require the intake susbsytem outside of the conditional command.  That will prevent
        // the conditional command from interrupting other commands if the intake is out.
        // When the intake is in, the subsystem requirements will take effect when ElevatorWristMoveToUpperPosition
        // is scheduled. 
        new ScheduleCommand(new ElevatorWristMoveToUpperPosition(ElevatorPosition.scoreHighCone.value, WristAngle.scoreMidHigh.value, elevator, wrist, intake, log)),
        new FileLogWrite(false, false, "Xbox Y", "Intake is out", log),
        () -> (!intake.isDeployed()) 
      ) // ONLY allow this command to run if the intake is in; otherwise would interrupt the ManipulatorGrab command  
    );
    
    //x
    // xbX.onTrue(new ElevatorSetPosition(ElevatorPosition.bottom, elevator, log));
    // Store elevator and wrist for traveling or pickup from conveyor
    xbX.onTrue(
      new ConditionalCommand(
        // Stow Elevator.  Schedule the elevator command separately, so this xbox button does not
        // require the intake susbsytem outside of the conditional command.  That will prevent
        // the conditional command from interrupting other commands if the intake is out.
        // When the intake is in, the subsystem requirements will take effect when ElevatorWristStow
        // is scheduled. 
        new ScheduleCommand(new ElevatorWristStow(elevator, wrist, log)),
        new FileLogWrite(false, false, "Xbox X", "Intake is out", log),
        () -> (!intake.isDeployed()) 
      ) // ONLY allow this command to run if the intake is in; otherwise would interrupt the ManipulatorGrab command   
    );        
    
    //lb
    xbLB.whileTrue(new ElevatorWristXboxControl(xboxController, elevator, wrist, log));     
    
    //rb
    // xbRB.onTrue(new ManipulatorSetPistonPosition(false, led, manipulator, log));     

    // Left Trigger
    // When held, manipulator choke up on game piece.  When released, manipulator go to holding power
    xbLT.onTrue(new ManipulatorSetPercent(ManipulatorConstants.pieceGrabPct, manipulator, log));
    xbLT.onFalse(new ManipulatorSetPercent(ManipulatorConstants.pieceHoldPct, manipulator, log));

    // Right Trigger
    // Score piece
    xbRT.onTrue(new EjectPiece(manipulator, log));

    //Right Bumber
    //Score piece (in case misclick)
    xbRB.onTrue(new EjectPiece(manipulator, log));
    // back
    // Turn off all motors
    xbBack.onTrue(Commands.parallel(
      new ManipulatorStopMotor(manipulator, log),
      new IntakeStop(intake, log)
      // new ConveyorMove(0, conveyor, log)
    )); 

    // start 
    // xbStart.onTrue(Command command); 

    // POV buttons

    // Up
    // Prepare to get cone
    xbPOVUp.onTrue(new ManipulatorSetPistonPosition(true, led, manipulator, log));

    // Down
    // Prepare to get cube
    xbPOVDown.onTrue(new ManipulatorSetPistonPosition(false, led, manipulator, log));

    // Left
    // Sets elevator/wrist to stowed, turn on conveyor, turn on manipulator to load piece
    // xbPOVLeft.onTrue();
    // xbPOVLeft.onTrue(new LoadPieceConveyor(elevator, wrist, manipulator, conveyor, log));

    // Right
    // Move elevator to loading station config and turn on manipulator to grab piece
    xbPOVRight.onTrue( new LoadPieceLoadingStation(elevator, wrist, manipulator, intake, log) );
  }

  /**
   * Define drivers joystick button mappings.
   */
  public void configureJoystickButtons() {
    JoystickButton[] left = new JoystickButton[3];
    JoystickButton[] right = new JoystickButton[3];

    for (int i = 1; i < left.length; i++) {
      left[i] = new JoystickButton(leftJoystick, i);
      right[i] = new JoystickButton(rightJoystick, i);
    }

    // If the robot angle drifts (or is turned on with the wrong facing), then this button can be used to 
    // reset the robot facing for field-oriented control.  Turn the robot so that it is facing away
    // from the driver, then press this button.

    // left joystick left button
    //left[1].onTrue(new IntakeRetractAndFlush(intakeFront, uptake, feeder, log));
    // resets current angle to 0, keeps current X and Y
    //left[1].onTrue(new DriveResetPose(0, false, driveTrain, log));
    left[1].onTrue(new EjectPiece(manipulator, log));
    // drive to closest goal
    left[2].whileTrue(
      new ConditionalCommand(
        new SequentialCommandGroup(
          new DriveToPose(() -> field.getInitialColumn(field.getClosestGoal(driveTrain.getPose(), manipulator.getPistonCone())), 0.15, 10, driveTrain, log),
          new DriveToPose(() -> field.getFinalColumn(field.getClosestGoal(driveTrain.getPose(), manipulator.getPistonCone())), driveTrain, log)
        ),
        new DriveToLoad(driveTrain, wrist, elevator, manipulator, intake, field, log),
        () -> driveTrain.getPose().getX() < 8
      )
    );
   
    // left joystick right button
    right[1].onTrue(
      // new IntakeExtendAndTurnOnMotors(manipulator, intake, wrist, elevator, led, log)
      new ConditionalCommand(
        // Extend intake.  Schedule the intake command separately, so this joystick button does not
        // require the Manipulator susbsytem outside of the conditional command.  That will prevent
        // the conditional command from interrupting other commands if the elevator is up.
        // When the elevator is down, the subsystem requirements will take effect when IntakeRetractAndTurnOffMotors
        // is scheduled. 
        new ScheduleCommand(new IntakeExtendAndTurnOnMotors(manipulator, intake, wrist, elevator, led, log)),
        new FileLogWrite(false, false, "RightJoystickButton1", "IntakeExtendAndTurnOnMotors - Elevator Not Down", log),
        () -> (elevator.getElevatorPos() <= Constants.ElevatorConstants.boundBottomMain) 
      ) // ONLY allow this command to run if elevator is not out; otherwise would interrupt the ManipulatorGrab command
    );

    
    right[2].onTrue(new IntakeRetractAndTurnOffMotors(intake, elevator, log));
    // right[1].onTrue(new DriveToPose(CoordType.kAbsolute, 0, driveTrain, log));
    // right[2].onTrue(new DriveToPose(CoordType.kRelative, 180, driveTrain, log));

    //left[2].onTrue(new IntakeRetractAndFlush(intakeFront, uptake, feeder, log));
      
    // right joystick left button
    // right[1].onTrue(new IntakeExtendAndTurnOnMotors(intakeFront, uptake, log)); 

    // right joystick right button
    // right[2].onTrue(new IntakeRetractAndFlush(intakeFront, uptake, feeder, log));  
  }

  /** 
   * Define Copanel button mappings.
   *  
   *  1  3  5  8
   *  2  4  6  8
   *      
   *  9  11 13 7
   *  10 12 14 7
   * 
   *  15
   *  16
   */
  public void configureCopanel() {
    JoystickButton[] coP = new JoystickButton[20];

    for (int i = 1; i < coP.length; i++) {
      coP[i] = new JoystickButton(coPanel, i);
    }

    // top row UP then DOWN, from LEFT to RIGHT
    coP[1].onTrue(new SequentialCommandGroup(
      new IntakeSetPercentOutput(-.5, -.25, intake, log),
      new ManipulatorSetPercent(-.5, manipulator, log)
    )); 
   coP[2].onTrue(new DriveResetPose(0, false, driveTrain, log));
    // coP[3].onTrue(new UptakeFeedBall(uptake, feeder, log)); 
    // coP[4].onTrue(new UptakeEjectBall(uptake, log)); 

    // coP[5].whenHeld(new ClimbSetPercentOutput(0.4, climb, log)); // manually raise climb arms, slowly
    // coP[6].whenHeld(new ClimbSetPercentOutput(-0.4, climb, log)); // manually lower climb arms, slowly
    
    // top row RED SWITCH
    // coP[8].onTrue(new StopAllMotors(feeder, shooter, intakeFront, uptake, log));

    // middle row UP then DOWN, from LEFT to RIGHT
    // coP[9].onTrue(new IntakeSetPercentOutput(IntakeConstants.onPct, IntakeConstants.onPctTransfer, intakeFront, log)); // forward intake and transfer
    // coP[10].onTrue(new IntakeSetPercentOutput(-IntakeConstants.onPct, -IntakeConstants.onPctTransfer, intakeFront, log)); // reverse intake and transfer

    // coP[11].onTrue(new UptakeSetPercentOutput(-UptakeConstants.onPct, 0, uptake, log)); // reverse uptake
    // coP[12].onTrue(new UptakeSetPercentOutput(UptakeConstants.onPct, 0, uptake, log)); // forward uptake

    // coP[13].onTrue(new FeederSetPercentOutput(-FeederConstants.onPct, feeder, log)); // reverse feeder
    // coP[14].onTrue(new FeederSetPercentOutput(FeederConstants.onPct, feeder, log)); // forward feeder

    // middle row UP OR DOWN, fourth button
    // coP[7].onTrue(new IntakePistonToggle(intakeFront, uptake, log)); 

    // bottom row UP then DOWN, from LEFT to RIGHT
    // coP[15].onTrue(new ClimberSetExtended(true,climber, log)); // climb extend
    // coP[16].onTrue(new ClimberSetExtended(false,climber, log)); // climb retract
  }


  /**
   * Sets the rumble on the XBox controller
   * @param percentRumble The normalized value (0 to 1) to set the rumble to
   */
	public void setXBoxRumble(double percentRumble) {
		xboxController.getHID().setRumble(RumbleType.kLeftRumble, percentRumble);
    xboxController.getHID().setRumble(RumbleType.kRightRumble, percentRumble);

    if (percentRumble == 0) rumbling = false;
    else rumbling = true;
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelection.getAutoCommand(intake, elevator, wrist, manipulator, driveTrain, led, log);
  }


  /**
   * Method called when robot is initialized.
   */
  public void robotInit() {
    SmartDashboard.putBoolean("RobotPrefs Initialized", RobotPreferences.prefsExist());
    if(!RobotPreferences.prefsExist()) {
      RobotPreferences.recordStickyFaults("RobotPreferences", log);
    }
    lastEnabledModeAuto = false;

    // compressor.disable();
    compressor.enableDigital();

    // Set initial robot position on field
    // This takes place a while after the drivetrain is created, so after any CanBus delays.
    driveTrain.resetPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));  
  }

  /**
   * robotPeriodic is run every 20msec
   */
  public void robotPeriodic(){
    log.advanceLogRotation();
    allianceSelection.periodic();
  }

  /**
   * Method called when robot is disabled.
   */
  public void disabledInit() {
    log.writeLogEcho(true, "Disabled", "Robot disabled");   // Don't log the word "Init" here -- it affects the Excel macro

    if (!lastEnabledModeAuto) {
      driveTrain.setDriveModeCoast(true);     // When pushing a disabled robot by hand, it is a lot easier to push in Coast mode!!!!
    }

    driveTrain.stopMotors();                // SAFETY:  Turn off any closed loop control that may be running, so the robot does not move when re-enabled.
    driveTrain.enableFastLogging(false);    // Turn off fast logging, in case it was left on from auto mode

    elevator.setMotorModeCoast(true);

    patternTeamMoving.schedule();
  }

  /**
   * Method called once every scheduler cycle when robot is disabled.
   */
  public void disabledPeriodic() {
    // Check for CAN bus error.  This is to prevent the issue that caused us to be eliminated in 2020!
    if (driveTrain.canBusError()) {
      RobotPreferences.recordStickyFaults("CAN Bus", log);
    }  //    TODO May want to flash this to the driver with some obvious signal!
    // boolean error = true;  
    // if (error == false) {
    //   if(!patternTeamMoving.isScheduled()) patternTeamMoving.schedule();
    // }
    // else {
    //   patternTeamMoving.cancel();
    //   led.setStrip("Red", 0.5, 0);
    // }
  }
  
  /**
   * Method called when auto mode is initialized/enabled.
   */
  public void autonomousInit() {
    log.writeLogEcho(true, "Auto", "Mode Init");
    lastEnabledModeAuto = true;

    driveTrain.setDriveModeCoast(false);
    driveTrain.resetGyroPitch();
    // driveTrain.cameraInit();
    elevator.setMotorModeCoast(false);

    if (patternTeamMoving.isScheduled()) patternTeamMoving.cancel();
    if (allianceSelection.getAlliance() == Alliance.Blue) {
      led.setStrip(Color.kBlue, 0);
    } else {
      led.setStrip(Color.kRed, 0);
    }
    // NOTE:  Do NOT reset the gyro or encoder here!!!!!
    // The first command in auto mode initializes before this code is run, and
    // it will read the gyro/encoder before the reset goes into effect.
  }

  /**
   * Method called once every scheduler cycle when auto mode is initialized/enabled
   */
  public void autonomousPeriodic() {
  }

  /**
   * Method called when teleop mode is initialized/enabled.
   */
  public void teleopInit() {
    log.writeLogEcho(true, "Teleop", "Mode Init");
    lastEnabledModeAuto = false;

    driveTrain.setDriveModeCoast(false);
    driveTrain.enableFastLogging(false);    // Turn off fast logging, in case it was left on from auto mode
    driveTrain.cameraInit();
    elevator.setMotorModeCoast(false);

    if (patternTeamMoving.isScheduled()) {
      patternTeamMoving.cancel();
      if (allianceSelection.getAlliance() == Alliance.Blue) {
        led.setStrip(Color.kBlue, 0);
      } else {
        led.setStrip(Color.kRed, 0);
      }
    }
  }

  /**
   * Method called once every scheduler cycle when teleop mode is initialized/enabled.
   */
  public void teleopPeriodic() {

  }
}
