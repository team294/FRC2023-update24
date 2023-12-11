package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.*;
import frc.robot.commands.autos.*;
import frc.robot.commands.sequences.*;
import frc.robot.subsystems.*;


/**
 * Selects the auto routine based upon input from the shuffleboard
 */
public class AutoSelection {

	public static final int NONE = 0;
	public static final int CONE_LEAVE_NEAR_WALL = 1;
	public static final int SCORE_CONE = 2;
	public static final int CONE_BALANCE_4TOWALL = 3;
	public static final int BALANCE_4TOWALL = 4;
	public static final int CONE_LEAVE_NEAR_WALL_CROSS = 5;
	public static final int CONE_LEAVE_NEAR_WALL_BALANCE = 6;
	public static final int CONE_LEAVE_NEAR_LOAD_BALANCE = 7;
	public static final int CONE_LEAVE_NEAR_WALL_PICK_UP_CUBE = 8;
	public static final int CONE_LEAVE_NEAR_LOAD_PICK_UP_CUBE = 9;
	public static final int CONE_LEAVE_NEAR_LOAD_PICK_UP_BALANCE = 10;
	public static final int CONE_LEAVE_NEAR_WALL_PICK_UP_BALANCE = 11;
	public static final int CONE_MOBILITY_BALANCE_4TOWALL = 12;
	public static final int CONE_PICK_UP_CUBE_BALANCE_4TOWALL = 13;
	public static final int CONE_PICK_UP_CUBE_SCORE_BALANCE_4TOWALL = 14;
	public static final int CONE_PICK_UP_CUBE_BALANCE_BARF_4TOWALL = 15;
	// 	public static final int LEAVE_COMMUNITY = 2;

	private final AllianceSelection allianceSelection;
	private final TrajectoryCache trajectoryCache;
	private final Field field;
	private SendableChooser<Integer> autoChooser = new SendableChooser<>();
	
	/**
	 * AutoSelection constructor for command group
	 * Sets up autoPlan widget 
	 */  	
	public AutoSelection(TrajectoryCache trajectoryCache, AllianceSelection allianceSelection, Field field, FileLog log) {
		this.trajectoryCache = trajectoryCache;
		this.allianceSelection = allianceSelection;
		this.field = field;

		// auto selections
		autoChooser.setDefaultOption("None", NONE);
		autoChooser.addOption("Score Cone", SCORE_CONE);
		autoChooser.addOption("Cone Leave NearWall", CONE_LEAVE_NEAR_WALL);
		autoChooser.addOption("Cone Balance 4ToWall", CONE_BALANCE_4TOWALL);
		autoChooser.addOption("Balance 4ToWall", BALANCE_4TOWALL);
		autoChooser.addOption("Cone Nearwall Cross", CONE_LEAVE_NEAR_WALL_CROSS);
		autoChooser.addOption("Cone Nearwall Balance", CONE_LEAVE_NEAR_WALL_BALANCE);
		autoChooser.addOption("Cone Near Load Balance", CONE_LEAVE_NEAR_LOAD_BALANCE);
		autoChooser.addOption("Cone Leave Near Wall pick up cube", CONE_LEAVE_NEAR_WALL_PICK_UP_CUBE);
		autoChooser.addOption("Cone Leave Near Load pick up cube", CONE_LEAVE_NEAR_LOAD_PICK_UP_CUBE);
		autoChooser.addOption("Cone Leave Near Wall Pick Up Balance", CONE_LEAVE_NEAR_WALL_PICK_UP_BALANCE);
		autoChooser.addOption("Cone Leave Near Load Pick Up Balance", CONE_LEAVE_NEAR_LOAD_PICK_UP_BALANCE);
		autoChooser.addOption("Cone Mobility Balance 4ToWall", CONE_MOBILITY_BALANCE_4TOWALL);
		autoChooser.addOption("Cone Pick Up Cube Balance 4ToWall", CONE_PICK_UP_CUBE_BALANCE_4TOWALL);
		autoChooser.addOption("Cone Pick Up Cube Score Balance 4ToWall", CONE_PICK_UP_CUBE_SCORE_BALANCE_4TOWALL);
		autoChooser.addOption("Cone Pick Up Cube Balance Barf 4ToWall", CONE_PICK_UP_CUBE_BALANCE_BARF_4TOWALL);

		
		// autoChooser.addOption("Straight", STRAIGHT);
		// autoChooser.addOption("Leave Community", LEAVE_COMMUNITY);
		// autoChooser.addOption("Right One Cone Balance", RIGHT_ONE_CONE_BALANCE);
		// autoChooser.addOption("Left One Cone Balance", LEFT_ONE_CONE_BALANCE);
		// autoChooser.addOption("Middle One Cone Balance", MIDDLE_ONE_CONE_BALANCE);
		// autoChooser.addOption("Middle Balance", MIDDLE_BALANCE);
	
		// show auto selection widget on Shuffleboard
		SmartDashboard.putData("Autonomous routine", autoChooser);

		// show auto parameters on Shuffleboard
		SmartDashboard.putNumber("Autonomous delay", 0);
		SmartDashboard.putBoolean("Autonomous use vision", false);
	}

	/**
	 * Gets the auto command based upon input from the shuffleboard.
	 * This method is designed to be called at AutonomousInit by Robot.java.
	 * 
	 * @param driveTrain The driveTrain that will be passed to the auto command
	 * @param log        The filelog to write the logs to
	 * @return the command to run
	 */
	public Command getAutoCommand(Intake intake, Elevator elevator, Wrist wrist, Manipulator manipulator, DriveTrain driveTrain, LED led, FileLog log) {
		Command autonomousCommand = null;

		// Get parameters from Shuffleboard
		int autoPlan = autoChooser.getSelected();
		log.writeLogEcho(true, "AutoSelect", "autoPlan",autoPlan);


		double waitTime = SmartDashboard.getNumber("Autonomous delay", 0);
		waitTime = MathUtil.clamp(waitTime, 0, 15);		// make sure autoDelay isn't negative and is only active during auto

		if (autoPlan == NONE) {
			// Starting position = facing drivers
			log.writeLogEcho(true, "AutoSelect", "run None");
			autonomousCommand = new DriveResetPose(180, false, driveTrain, log);
		}

		if (autoPlan == SCORE_CONE) {
			// Starting position = facing drivers, against a cone scoring location
			log.writeLogEcho(true, "AutoSelect", "run Score Cone");
	   		autonomousCommand = new SequentialCommandGroup(
				new WaitCommand(waitTime),
			   	new DriveResetPose(180, false, driveTrain, log),
				new AutoScoreConeHigh(true, elevator, wrist, manipulator, intake, led, log)
	   		);
   	   	}

		if (autoPlan == CONE_LEAVE_NEAR_WALL) {
			// Starting position = facing drivers, against scoring position closest to wall
			log.writeLogEcho(true, "AutoSelect", "run Cone Leave Near Wall");
			Pose2d posScoreInitial, posLeaveFinal;
			if (allianceSelection.getAlliance() == Alliance.Red) {
				posScoreInitial = field.getFinalColumn(9);
			} else {
				posScoreInitial = field.getFinalColumn(1);
			}
			// Travel  4.4 m in +X from starting position
			posLeaveFinal = MathBCR.translate(posScoreInitial, 4.4, 0);

	   		autonomousCommand = new SequentialCommandGroup(
				new WaitCommand(waitTime),
			    new DriveResetPose(posScoreInitial, true, driveTrain, log),
			    new AutoScoreConeHigh(true, elevator, wrist, manipulator, intake, led, log),
				new DriveToPose(posLeaveFinal, driveTrain, log)
	   		);
   	   	}

		if (autoPlan == CONE_LEAVE_NEAR_WALL_CROSS) {
			// Starting position = facing drivers, against scoring position closest to wall
			log.writeLogEcho(true, "AutoSelect", "run Cone Leave Near Wall Cross");
			Pose2d posScoreInitial, posLeave, posCross, posFinal;
			if (allianceSelection.getAlliance() == Alliance.Red) {
				posScoreInitial = field.getFinalColumn(9);			// 1.77165, 7.490968, 180
				// Travel  4.4 m in +X from starting position
				posLeave = MathBCR.translate(posScoreInitial, 4.4, 0);  // 6.17165, 7.490968, 180
				// Travel in Y to cross the field to the pickup side
				posCross = new Pose2d(6.3, 2.2, Rotation2d.fromDegrees(180.0));
				// Spin 180
				posFinal = new Pose2d(7.0, 2.2, Rotation2d.fromDegrees(0.0));
			} else {
				posScoreInitial = field.getFinalColumn(1);			// 1.77165, 0.512826, 180
				// Travel  4.4 m in +X from starting position
				posLeave = MathBCR.translate(posScoreInitial, 4.4, 0);		// 6.17165, 0.512826, 180
				// Travel in Y to cross the field to the pickup side
				posCross = new Pose2d(6.3, 6.0, Rotation2d.fromDegrees(180.0));
				// Spin 180
				posFinal = new Pose2d(7.0, 6.0, Rotation2d.fromDegrees(0.0));
			}

	   		autonomousCommand = new SequentialCommandGroup(
				new WaitCommand(waitTime),
			    new DriveResetPose(posScoreInitial, true, driveTrain, log),
			    new AutoScoreConeHigh(false, elevator, wrist, manipulator, intake, led, log),
				new ParallelCommandGroup(
					new ElevatorWristStow(elevator, wrist, log),
					new DriveToPose(posLeave, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kMaxRetractingAccelerationMetersPerSecondSquare,
					TrajectoryConstants.interimPositionErrorMeters, TrajectoryConstants.interimThetaErrorDegrees, driveTrain, log)
				),
				new DriveToPose(posLeave, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare,
					TrajectoryConstants.interimPositionErrorMeters, TrajectoryConstants.interimThetaErrorDegrees, driveTrain, log),
				new DriveToPose(posCross, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare,
				TrajectoryConstants.interimPositionErrorMeters, TrajectoryConstants.interimThetaErrorDegrees, driveTrain, log),
				new DriveToPose(posFinal, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare,
					TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, driveTrain, log)
	   		);
   	   	}

		if (autoPlan == CONE_LEAVE_NEAR_WALL_BALANCE) {
			// Starting position = facing drivers, against scoring position closest to wall
			log.writeLogEcho(true, "AutoSelect", "run Cone Leave Near Wall Balance");
			Pose2d posScoreInitial, posLeave, posCross;
			if (allianceSelection.getAlliance() == Alliance.Red) {
				posScoreInitial = field.getFinalColumn(9);			// 1.77165, 7.490968, 180
				// Travel  4.4 m in +X from starting position
				posLeave = MathBCR.translate(posScoreInitial, 4, 0);  // 6.17165, 7.490968, 180
				// Travel in Y to cross the field to in front of charging station
				// posCross = new Pose2d(6.3, 2.2, Rotation2d.fromDegrees(180.0));
				posCross = MathBCR.translate(field.getStationInitial(5), 1, 0);
				// Spin 180
				// posFinal = field.getStationCenter(2);
			} else {
				posScoreInitial = field.getFinalColumn(1);			// 1.77165, 0.512826, 180
				// Travel  3.5 m in +X from starting position
				posLeave = MathBCR.translate(posScoreInitial, 4, 0);		// 6.17165, 0.512826, 180
				// Travel in Y to cross the field to the in front of charging station
				posCross = MathBCR.translate(field.getStationInitial(5), 1, 0);
			}
				
			autonomousCommand = new SequentialCommandGroup(
				new SequentialCommandGroup(
					new WaitCommand(waitTime),
					new DriveResetPose(posScoreInitial, true, driveTrain, log),
					new AutoScoreConeHigh(false, elevator, wrist, manipulator, intake, led, log),
					// new DriveToPose(posLeave, SwerveConstants.kMaxSpeedMetersPerSecond, SwerveConstants.kMaxAccelerationMetersPerSecondSquare,
					// 	TrajectoryConstants.interimPositionErrorMeters, TrajectoryConstants.interimThetaErrorDegrees,false, driveTrain, log),
					new ParallelCommandGroup(
						new ElevatorWristStow(elevator, wrist, log),
						new DriveToPose(posLeave, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kMaxRetractingAccelerationMetersPerSecondSquare,
							TrajectoryConstants.interimPositionErrorMeters, TrajectoryConstants.interimThetaErrorDegrees, driveTrain, log)
					),
					new DriveToPose(posCross, SwerveConstants.kFullSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare,
						0.4, TrajectoryConstants.interimThetaErrorDegrees, driveTrain, log),
					// new DriveToPose(posCross, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare,
					// 	TrajectoryConstants.interimPositionErrorMeters, TrajectoryConstants.interimThetaErrorDegrees, false, driveTrain, log),0
					new FileLogEnableFastLogging(true, driveTrain, log),
					new DriveUpChargingStation(-TrajectoryConstants.ChargeStationVelocity, 0.6, 3.5, driveTrain, log),
					// new ActiveBalance(driveTrain, log)
					new ActiveBalanceTwo(driveTrain, log)
				)
				// .withTimeout(14.5),
				// new FileLogEnableFastLogging(false, driveTrain, log),
				// new DriveToPose(CoordType.kRelative, 0.5, driveTrain, log)		// Lock the wheels at 45deg
			);
		}

		if (autoPlan == CONE_LEAVE_NEAR_LOAD_BALANCE) {
			// Starting position = facing drivers, against scoring position closest to wall
			log.writeLogEcho(true, "AutoSelect", "run Cone Leave Near Load Balance");
			Pose2d posScoreInitial, posLeave, posCross;
			if (allianceSelection.getAlliance() == Alliance.Red) {
				posScoreInitial = field.getFinalColumn(1);			// 1.77165, 7.490968, 180
				// Travel  4.4 m in +X from starting position
				posLeave = MathBCR.translate(posScoreInitial, 4, 0);  // 6.17165, 7.490968, 180
				// Travel in Y to cross the field to in front of charging station
				// posCross = new Pose2d(6.3, 2.2, Rotation2d.fromDegrees(180.0));
				posCross = MathBCR.translate(field.getStationInitial(5), 1, 0);
				// Spin 180
				// posFinal = field.getStationCenter(2);
			} else {
				posScoreInitial = field.getFinalColumn(9);			// 1.77165, 0.512826, 180
				// Travel  3.5 m in +X from starting position
				posLeave = MathBCR.translate(posScoreInitial, 4, 0);		// 6.17165, 0.512826, 180
				// Travel in Y to cross the field to the in front of charging station
				posCross = MathBCR.translate(field.getStationInitial(5), 1, 0);
			}
				
			autonomousCommand = new SequentialCommandGroup(
				new SequentialCommandGroup(
					new WaitCommand(waitTime),
					new DriveResetPose(posScoreInitial, true, driveTrain, log),
					new AutoScoreConeHigh(false, elevator, wrist, manipulator, intake, led, log),
					new ParallelCommandGroup(
						new ElevatorWristStow(elevator, wrist, log),
						new DriveToPose(posLeave, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kMaxRetractingAccelerationMetersPerSecondSquare,
							TrajectoryConstants.interimPositionErrorMeters, TrajectoryConstants.interimThetaErrorDegrees, driveTrain, log)
					),
					// new DriveToPose(posLeave, SwerveConstants.kMaxSpeedMetersPerSecond, SwerveConstants.kMaxAccelerationMetersPerSecondSquare,
					// 	TrajectoryConstants.interimPositionErrorMeters, TrajectoryConstants.interimThetaErrorDegrees, false, driveTrain, log),
					new DriveToPose(posCross, SwerveConstants.kFullSpeedMetersPerSecond, SwerveConstants.kFullAccelerationMetersPerSecondSquare,
						0.4, TrajectoryConstants.interimThetaErrorDegrees, driveTrain, log),
					new FileLogEnableFastLogging(true, driveTrain, log),
					new DriveUpChargingStation(-TrajectoryConstants.ChargeStationVelocity, 0.6, 3.5, driveTrain, log),
					// new ActiveBalance(driveTrain, log)
					new ActiveBalanceTwo(driveTrain, log)
				)
				// .withTimeout(14.5),
				// new FileLogEnableFastLogging(false, driveTrain, log),
				// new DriveToPose(CoordType.kRelative, 0.5, driveTrain, log)		// Lock the wheels at 45deg
			);
		}

		if (autoPlan == CONE_LEAVE_NEAR_WALL_PICK_UP_CUBE) {
			// Starting position = facing drivers, against scoring position closest to wall
			log.writeLogEcho(true, "AutoSelect", "run Cone Leave Near Wall Pickup cube");
			Pose2d posScoreInitial, posLeave, posLineUp, posFinal;
			if (allianceSelection.getAlliance() == Alliance.Red) {
				posScoreInitial = field.getFinalColumn(9);			// 1.77165, 7.490968, 180
				// Travel  4.4 m in +X from starting position
				posLeave = MathBCR.translate(posScoreInitial, 5.39, -.407174);  // 6.17165, 7.490968, 180
				// Travel in Y to cross the field to in front of charging station
				// posCross = new Pose2d(6.3, 2.2, Rotation2d.fromDegrees(180.0));
				posLineUp = field.getInitialColumn(8);
				posFinal = field.getFinalColumn(8);
				// Spin 180
				// posFinal = field.getStationCenter(2);
			} else {
				posScoreInitial = field.getFinalColumn(1);			// 1.77165, 0.512826, 180
				// Travel  3.5 m in +X from starting position
				posLeave = MathBCR.translate(posScoreInitial, 5.39, .407174);		
				// Travel in Y to cross the field to the in front of charging station
				posLineUp = field.getInitialColumn(2);
				posFinal = field.getFinalColumn(2);
			}
				
			autonomousCommand = new SequentialCommandGroup(
				new WaitCommand(waitTime),
				new DriveResetPose(posScoreInitial, true, driveTrain, log),
				new AutoScoreConeHigh(false, elevator, wrist, manipulator, intake, led, log),
				new AutoPickUpCube(posLeave, posLineUp, true, intake, elevator, wrist, manipulator, driveTrain, led, log),
				new AutoScoreCube(posFinal, driveTrain, elevator, wrist, manipulator, intake, led, log),
				new DriveToPose(posLineUp, driveTrain, log)
			);
		}

		if (autoPlan == CONE_LEAVE_NEAR_LOAD_PICK_UP_CUBE) {
			// Starting position = facing drivers, against scoring position closest to wall
			log.writeLogEcho(true, "AutoSelect", "run Cone Leave Near Load Pick up cube");
			Pose2d posScoreInitial, posCube, posMidPoint, posScore, posEnd;
			if (allianceSelection.getAlliance() == Alliance.Red) {
				posScoreInitial = field.getFinalColumn(1);			// 1.77165, 7.490968, 180
				// Travel  4.4 m in +X from starting position
				posCube = MathBCR.translate(posScoreInitial, 5.39, .407174);  // 6.17165, 7.490968, 180
				// Travel in Y to cross the field to in front of charging station
				// posCross = new Pose2d(6.3, 2.2, Rotation2d.fromDegrees(180.0));
				posMidPoint = MathBCR.translate(posScoreInitial, 1.5, 0.25);			// 1.5, .25  : Add a little room to clear between charging station and loading area
				// posScore = field.getFinalColumn(2);
				posScore = MathBCR.translate(field.getFinalColumn(2), 0, -0.2);		// G7:  Move scoring location by 0.2m to better center cube
				posEnd = new Pose2d( posCube.getTranslation(), posCube.getRotation().plus(Rotation2d.fromDegrees(90)) );
				// Spin 180
				// posFinal = field.getStationCenter(2);
			} else {
				posScoreInitial = field.getFinalColumn(9);			// 1.77165, 0.512826, 180
				// Travel  3.5 m in +X from starting position
				posCube = MathBCR.translate(posScoreInitial, 5.39, -.207174);		// G5:  was 5.39, -.407174.  Change to 5.39, -.207174 to prevent cube from hitting elevator IGUS
				// Travel in Y to cross the field to the in front of charging station
				posMidPoint = MathBCR.translate(posScoreInitial, 1.5, -.25);			// 1.5, .25  : Add a little room to clear between charging station and loading area
				// posScore = field.getFinalColumn(8);
				posScore = MathBCR.translate(field.getFinalColumn(8), 0, 0.2);		// G5:  Move scoring location by 0.2m to better center cube
				posEnd = new Pose2d( posCube.getTranslation(), posCube.getRotation().plus(Rotation2d.fromDegrees(-90)) );
			}
				
			autonomousCommand = new SequentialCommandGroup(
				new WaitCommand(waitTime),
				new DriveResetPose(posScoreInitial, true, driveTrain, log),
				new AutoScoreConeHigh(false, elevator, wrist, manipulator, intake, led, log),
				new AutoPickupCubeAndSetScore(posCube, posMidPoint, posScore, true, intake, elevator, wrist, manipulator, driveTrain, led, log),

				new ConditionalCommand(
					// If we have a cube in the manipulator, then score and then drive out to get the next cube
					new SequentialCommandGroup(
						// Go towards getting next piece
						new EjectPiece(manipulator, log), 		// Runs for 1 second
						new ElevatorWristStow(elevator, wrist, log),
						new DriveToPose(posMidPoint, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kFullAccelerationMetersPerSecondSquare,
							TrajectoryConstants.interimPositionErrorMeters, TrajectoryConstants.interimThetaErrorDegrees, driveTrain, log),
						new DriveToPose(posCube, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kFullAccelerationMetersPerSecondSquare,
							TrajectoryConstants.interimPositionErrorMeters, TrajectoryConstants.interimThetaErrorDegrees, driveTrain, log),
						new DriveToPose(posEnd, driveTrain, log)
					),

					// If we don't have a cube in the manipulator, then just try score and stay at the scoring area
					new SequentialCommandGroup(
						new EjectPiece(manipulator, log), 		// Runs for 1 second
						new ElevatorWristStow(elevator, wrist, log),
						new WaitCommand(0.01)
					),

					() -> manipulator.isCubePresent()
				)

			);
		}

		if (autoPlan == CONE_LEAVE_NEAR_WALL_PICK_UP_BALANCE) {
			// Starting position = facing drivers, against scoring position closest to wall
			log.writeLogEcho(true, "AutoSelect", "run Cone Leave Near Wall pickup balance");
			Pose2d posScoreInitial, posLeave, posCross;
			if (allianceSelection.getAlliance() == Alliance.Red) {
				posScoreInitial = field.getFinalColumn(9);			// 1.77165, 7.490968, 180
				// Travel  4.4 m in +X from starting position
				posLeave = MathBCR.translate(posScoreInitial, 5.39, -.407174);  // 6.17165, 7.490968, 180
				// Travel in Y to cross the field to in front of charging station
				// posCross = new Pose2d(6.3, 2.2, Rotation2d.fromDegrees(180.0));
				posCross = MathBCR.translate(field.getStationInitial(5), 1, 0);
				// Spin 180
				// posFinal = field.getStationCenter(2);
			} else {
				posScoreInitial = field.getFinalColumn(1);			// 1.77165, 0.512826, 180
				// Travel  3.5 m in +X from starting position
				posLeave = MathBCR.translate(posScoreInitial, 5.39, .407174);		// 6.17165, 0.512826, 180
				// Travel in Y to cross the field to the in front of charging station
				posCross = MathBCR.translate(field.getStationInitial(5), 1, 0);
			}
				
			autonomousCommand = new SequentialCommandGroup(
				new SequentialCommandGroup(
					new WaitCommand(waitTime),
					new DriveResetPose(posScoreInitial, true, driveTrain, log),
					new AutoScoreConeHigh(false, elevator, wrist, manipulator, intake, led, log),
					new AutoPickUpCube(posLeave, posCross, false, intake, elevator, wrist, manipulator, driveTrain, led, log),
					new FileLogEnableFastLogging(true, driveTrain, log),
					new DriveUpChargingStation(-TrajectoryConstants.ChargeStationVelocity, 0.6, 3.5, driveTrain, log),
					// new ActiveBalance(driveTrain, log)
					new ActiveBalanceTwo(driveTrain, log)
				)
				// .withTimeout(14.5),
				// new FileLogEnableFastLogging(false, driveTrain, log),
				// new DriveToPose(CoordType.kRelative, 0.5, driveTrain, log)		// Lock the wheels at 45deg
			);
		}

		if (autoPlan == CONE_LEAVE_NEAR_LOAD_PICK_UP_BALANCE) {
			// Starting position = facing drivers, against scoring position closest to wall
			log.writeLogEcho(true, "AutoSelect", "run Cone Leave load pickup balance");
			Pose2d posScoreInitial, posLeave, posCross;
			if (allianceSelection.getAlliance() == Alliance.Red) {
				posScoreInitial = field.getFinalColumn(1);			// 1.77165, 7.490968, 180
				// Travel  4.4 m in +X from starting position
				posLeave = MathBCR.translate(posScoreInitial, 5.39, .407174);  // 6.17165, 7.490968, 180
				// Travel in Y to cross the field to in front of charging station
				// posCross = new Pose2d(6.3, 2.2, Rotation2d.fromDegrees(180.0));
				posCross = MathBCR.translate(field.getStationInitial(5), 1, 0);
				// Spin 180
				// posFinal = field.getStationCenter(2);
			} else {
				posScoreInitial = field.getFinalColumn(9);			// 1.77165, 0.512826, 180
				// Travel  3.5 m in +X from starting position
				posLeave = MathBCR.translate(posScoreInitial, 5.39, -.407174);		// 6.17165, 0.512826, 180
				// Travel in Y to cross the field to the in front of charging station
				posCross = MathBCR.translate(field.getStationInitial(5), 1, 0);
			}
				
			autonomousCommand = new SequentialCommandGroup(
				new SequentialCommandGroup(
					new WaitCommand(waitTime),
					new DriveResetPose(posScoreInitial, true, driveTrain, log),
					new AutoScoreConeHigh(false, elevator, wrist, manipulator, intake, led, log),
					new AutoPickUpCube(posLeave, posCross, false, intake, elevator, wrist, manipulator, driveTrain, led, log),
					new FileLogEnableFastLogging(true, driveTrain, log),
					new DriveUpChargingStation(-TrajectoryConstants.ChargeStationVelocity, 0.6, 3.5, driveTrain, log),
					// new ActiveBalance(driveTrain, log)
					new ActiveBalanceTwo(driveTrain, log)
				)
				// .withTimeout(14.5),
				// new FileLogEnableFastLogging(false, driveTrain, log),
				// new DriveToPose(CoordType.kRelative, 0.5, driveTrain, log)		// Lock the wheels at 45deg
			);
		}

		if (autoPlan == CONE_BALANCE_4TOWALL) {
			// Starting position = facing drivers, 4th scoring position from wall
			log.writeLogEcho(true, "AutoSelect", "run Cone Balance 4ToWall");
			// Pose2d posCommunityInitial = field.getStationInitial(2);
			// Pose2d posCommunityFinal = field.getStationCenter(2);
			// Pose2d posCommunityFinal = translate(field.getStationCenter(2), 1.5, 0.0);		// overdrive due to wheel slip when climbing on charging station
			Pose2d posScoreInitial;
			if (allianceSelection.getAlliance() == Alliance.Red) {
				posScoreInitial = field.getFinalColumn(6);
			} else {
				posScoreInitial = field.getFinalColumn(4);
			}

	   		autonomousCommand = new SequentialCommandGroup(
				new SequentialCommandGroup(
					new WaitCommand(waitTime),
					new DriveResetPose(posScoreInitial, true, driveTrain, log),
					new AutoScoreConeHigh(true, elevator, wrist, manipulator, intake, led, log),
					new FileLogEnableFastLogging(true, driveTrain, log),
					new DriveUpChargingStation(TrajectoryConstants.ChargeStationVelocity, 0.6, 3.9, driveTrain, log),		// Was 2.1m but overshot slightly in Qual39, trying 1.9m
					//new ActiveBalance(driveTrain, log)
					new ActiveBalanceTwo(driveTrain, log)
				)
			);
				// ).withTimeout(14.5),
				// new FileLogEnableFastLogging(false, driveTrain, log),
				// new DriveToPose(CoordType.kRelative, 0.5, driveTrain, log)		// Lock the wheels at 45deg
				// );
   	   	}

		if (autoPlan == BALANCE_4TOWALL) {
			// Starting position = facing drivers, 4th scoring position from wall
			log.writeLogEcho(true, "AutoSelect", "run Balance 4ToWall");
			
			Pose2d posScoreInitial;
			if (allianceSelection.getAlliance() == Alliance.Red) {
				posScoreInitial = field.getFinalColumn(6);
			} else {
				posScoreInitial = field.getFinalColumn(4);
			}

	   		autonomousCommand = new SequentialCommandGroup(
				new SequentialCommandGroup(
					new WaitCommand(waitTime),
					new DriveResetPose(posScoreInitial, true, driveTrain, log),
					new FileLogEnableFastLogging(true, driveTrain, log),
					new DriveUpChargingStation(TrajectoryConstants.ChargeStationVelocity, 0.6, 3.9, driveTrain, log),	// Changed from 2.1m to 1.9m after Qual39 (see CONE_BALANCE_4TOWALL)
					new ActiveBalanceTwo(driveTrain, log)
					//new ActiveBalance(driveTrain, log)
				)
				// ).withTimeout(14.5),
				// new FileLogEnableFastLogging(false, driveTrain, log),
				// new DriveToPose(CoordType.kRelative, 0.5, driveTrain, log)		// Lock the wheels at 45deg
	   		);
   	   	}

		if (autoPlan == CONE_MOBILITY_BALANCE_4TOWALL) {
		// Starting position = facing drivers, 4th scoring position from wall
			log.writeLogEcho(true, "AutoSelect", "run Cone Balance 4ToWall");
			// Pose2d posCommunityInitial = field.getStationInitial(2);
			// Pose2d posCommunityFinal = field.getStationCenter(2);
			// Pose2d posCommunityFinal = translate(field.getStationCenter(2), 1.5, 0.0);		// overdrive due to wheel slip when climbing on charging station
			Pose2d posScoreInitial;
			if (allianceSelection.getAlliance() == Alliance.Red) {
				posScoreInitial = field.getFinalColumn(6);
			} else {
				posScoreInitial = field.getFinalColumn(4);
			}

				autonomousCommand = new SequentialCommandGroup(
				new SequentialCommandGroup(
					new WaitCommand(waitTime),
					new DriveResetPose(posScoreInitial, true, driveTrain, log),
					new AutoScoreConeHigh(true, elevator, wrist, manipulator, intake, led, log),
					new FileLogEnableFastLogging(true, driveTrain, log),
					new DriveUpChargingStation(TrajectoryConstants.ChargeStationVelocity, 0.6, 3.9, driveTrain, log),		// Was 2.1m but overshot slightly in Qual39, trying 1.9m
					new DriveDownChargingStation(TrajectoryConstants.ChargeStationVelocity, 0.5, driveTrain, log),
					new DriveUpChargingStation(-TrajectoryConstants.ChargeStationVelocity, 0.6, 3.4, driveTrain, log),
					new ActiveBalanceTwo(driveTrain, log)
				)
			);
		}

		if (autoPlan == CONE_PICK_UP_CUBE_BALANCE_4TOWALL) {
			// Starting position = facing drivers, 4th scoring position from wall
			log.writeLogEcho(true, "AutoSelect", "run Cone Balance 4ToWall");
			// Pose2d posCommunityInitial = field.getStationInitial(2);
			// Pose2d posCommunityFinal = field.getStationCenter(2);
			// Pose2d posCommunityFinal = translate(field.getStationCenter(2), 1.5, 0.0);		// overdrive due to wheel slip when climbing on charging station
			Pose2d posScoreInitial, posPickUp, posNext;
			if (allianceSelection.getAlliance() == Alliance.Red) {
				posScoreInitial = field.getFinalColumn(6);
				posPickUp = MathBCR.translate(posScoreInitial, 5.39 + 0.4, 0);	
				posNext = MathBCR.translate(posPickUp, -.4, 0);
			} else {
				posScoreInitial = field.getFinalColumn(4);
				posPickUp = MathBCR.translate(posScoreInitial, 5.39+0.4, 0);	
				posNext = MathBCR.translate(posPickUp, -.4, 0);
				
			}

				autonomousCommand = new SequentialCommandGroup(
				new SequentialCommandGroup(
					new WaitCommand(waitTime),
					new DriveResetPose(posScoreInitial, true, driveTrain, log),
					new AutoScoreConeHigh(true, elevator, wrist, manipulator, intake, led, log),
					new FileLogEnableFastLogging(true, driveTrain, log),
					new DriveUpChargingStation(TrajectoryConstants.ChargeStationVelocity*1.5, 0.6, 3.9, driveTrain, log),		// Was 2.1m but overshot slightly in Qual39, trying 1.9m
					new DriveDownChargingStation(TrajectoryConstants.ChargeStationVelocity, 0.0, driveTrain, log),
					new AutoPickUpCube(posPickUp, posNext, false, intake, elevator, wrist, manipulator, driveTrain, led, log),
					new DriveUpChargingStation(-TrajectoryConstants.ChargeStationVelocity*1.4, 0.6, 3.4, driveTrain, log),
					new ActiveBalanceTwo(driveTrain, log)
				)
			);
		}

		if (autoPlan == CONE_PICK_UP_CUBE_SCORE_BALANCE_4TOWALL) {
			// Starting position = facing drivers, 4th scoring position from wall
			log.writeLogEcho(true, "AutoSelect", "run Cone Balance 4ToWall");
			// Pose2d posCommunityInitial = field.getStationInitial(2);
			// Pose2d posCommunityFinal = field.getStationCenter(2);
			// Pose2d posCommunityFinal = translate(field.getStationCenter(2), 1.5, 0.0);		// overdrive due to wheel slip when climbing on charging station
			Pose2d posScoreInitial, posPickUp, posNext, posScoreCube;
			if (allianceSelection.getAlliance() == Alliance.Red) {
				posScoreInitial = field.getFinalColumn(6);
				posPickUp = MathBCR.translate(posScoreInitial, 5.39 + 0.4, 0);	
				posNext = MathBCR.translate(posPickUp, -.4, 0);
				posScoreCube = field.getFinalColumn(6);
			} else {
				posScoreInitial = field.getFinalColumn(4);
				posPickUp = MathBCR.translate(posScoreInitial, 5.39 + 0.4, 0);	
				posNext = MathBCR.translate(posPickUp, -.4, 0);
				posScoreCube = field.getFinalColumn(4);

			}

				autonomousCommand = new SequentialCommandGroup(
				new SequentialCommandGroup(
					new WaitCommand(waitTime),
					new DriveResetPose(posScoreInitial, true, driveTrain, log),
					new AutoScoreConeHigh(true, elevator, wrist, manipulator, intake, led, log),
					new FileLogEnableFastLogging(true, driveTrain, log),
					new DriveUpChargingStation(TrajectoryConstants.ChargeStationVelocity * 1.5, 0.6, 3.9, driveTrain, log),		// Was 2.1m but overshot slightly in Qual39, trying 1.9m
					new DriveDownChargingStation(TrajectoryConstants.ChargeStationVelocity * 1.3, 0, driveTrain, log),
					new AutoPickUpCube(posPickUp, posNext, false, intake, elevator, wrist, manipulator, driveTrain, led, log),
					new DriveUpChargingStation(-TrajectoryConstants.ChargeStationVelocity * 1.5, 0.6, 3.4, driveTrain, log),
					new DriveDownChargingStation(-TrajectoryConstants.ChargeStationVelocity * 1, 0, driveTrain, log),
					new ElevatorWristMoveToUpperPosition(ElevatorPosition.scoreLow.value, WristAngle.scoreMidHigh.value, elevator, wrist, intake, log),
					new EjectPiece(1, 0.2, manipulator, log),
					new ParallelCommandGroup(
						new ElevatorWristStow(elevator, wrist, log),
						//new AutoScoreCube(posScoreCube, driveTrain, elevator, wrist, manipulator, intake, led, log), 
						new DriveUpChargingStation(TrajectoryConstants.ChargeStationVelocity, 0.6, 3.9, driveTrain, log)
					),
					new ActiveBalanceTwo(driveTrain, log)
				)
			);
		}

		if (autoPlan == CONE_PICK_UP_CUBE_BALANCE_BARF_4TOWALL) {
			// Starting position = facing drivers, 4th scoring position from wall
			log.writeLogEcho(true, "AutoSelect", "run Cone Pick up Cube Balance Barf 4ToWall");
			// Pose2d posCommunityInitial = field.getStationInitial(2);
			// Pose2d posCommunityFinal = field.getStationCenter(2);
			// Pose2d posCommunityFinal = translate(field.getStationCenter(2), 1.5, 0.0);		// overdrive due to wheel slip when climbing on charging station
			Pose2d posScoreInitial, posPickUp, posNext, posScoreCube;
			if (allianceSelection.getAlliance() == Alliance.Red) {
				posScoreInitial = field.getFinalColumn(6);
				posPickUp = MathBCR.translate(posScoreInitial, 5.39 + 0.4, 0);	
				posNext = MathBCR.translate(posPickUp, -.4, 0);
				posScoreCube = field.getFinalColumn(6);
			} else {
				posScoreInitial = field.getFinalColumn(4);
				posPickUp = MathBCR.translate(posScoreInitial, 5.39+0.4, 0);	
				posNext = MathBCR.translate(posPickUp, -.4, 0);
				posScoreCube = field.getFinalColumn(4);

			}

				autonomousCommand = new SequentialCommandGroup(
				new SequentialCommandGroup(
					new WaitCommand(waitTime),
					new DriveResetPose(posScoreInitial, true, driveTrain, log),
					new AutoScoreConeHigh(true, elevator, wrist, manipulator, intake, led, log),
					new FileLogEnableFastLogging(true, driveTrain, log),
					new DriveUpChargingStation(TrajectoryConstants.ChargeStationVelocity*1.5, 0.6, 3.9, driveTrain, log),		// Was 2.1m but overshot slightly in Qual39, trying 1.9m
					new DriveDownChargingStation(TrajectoryConstants.ChargeStationVelocity, 0, driveTrain, log),
					new AutoPickUpCube(posPickUp, posNext, false, intake, elevator, wrist, manipulator, driveTrain, led, log),
					new DriveUpChargingStation(-TrajectoryConstants.ChargeStationVelocity * 1.4, 0.6, 3.4, driveTrain, log),
					new ParallelCommandGroup(
						new SequentialCommandGroup(
							// new WaitCommand(0.5),
							new ElevatorWristMoveToUpperPosition(ElevatorPosition.scoreLow.value, WristAngle.scoreMidHigh.value, elevator, wrist, intake, log),
							new EjectPiece(1, 0.5, manipulator, log),
							new ElevatorWristStow(elevator, wrist, log)
						),
						new ActiveBalanceTwo(driveTrain, log)
	
					)
				).withTimeout(14.75),
				new FileLogEnableFastLogging(false, driveTrain, log),
				new DriveToPose(CoordType.kRelative, 0.5, driveTrain, log)	
				
			);
		}

		// 	if (autoPlan == STRAIGHT) {
	// 		log.writeLogEcho(true, "AutoSelect", "run Straight");
	// 	   autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime), 
	// 	   		new DriveTrajectory(CoordType.kRelative, StopType.kBrake, 
	// 					trajectoryCache.cache[TrajectoryType.test.value], driveTrain, log)
	// 	   );
	//    }

	//    if (autoPlan == LEAVE_COMMUNITY) {
	// 		log.writeLogEcho(true, "AutoSelect", "run Leave Community");
	//    		autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime),
	// 			new DriveTrajectory(CoordType.kAbsoluteResetPose, StopType.kBrake, 
	// 					trajectoryCache.cache[TrajectoryType.LeaveCommunity.value], driveTrain, log)
	//    		);
   	//    }

	//    if(autoPlan == RIGHT_ONE_CONE_BALANCE){
	// 		log.writeLogEcho(true, "AutoSelect", "run Right One Cone Balance");
	// 		autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime),
	// 			new DriveTrajectory(CoordType.kAbsoluteResetPoseTol, StopType.kBrake, 
	// 				((allianceSelection.getAlliance() == Alliance.Red) ? 
	// 					trajectoryCache.cache[TrajectoryType.RightOuterOneConeBalanceRed.value]:
	// 					trajectoryCache.cache[TrajectoryType.RightOuterOneConeBalanceBlue.value] ), 
	// 				driveTrain, log)
	// 		);
	//    }

	//    if(autoPlan == LEFT_ONE_CONE_BALANCE){
	// 		log.writeLogEcho(true, "AutoSelect", "run Left One Cone Balance");
	// 		autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime),
	// 			new DriveTrajectory(CoordType.kAbsoluteResetPoseTol, StopType.kBrake, 
	// 				((allianceSelection.getAlliance() == Alliance.Red) ? 
	// 					trajectoryCache.cache[TrajectoryType.LeftOuterOneConeBalanceRed.value]:
	// 					trajectoryCache.cache[TrajectoryType.LeftOuterOneConeBalanceBlue.value] ), 
	// 				driveTrain, log)
	// 		);
	//    }

	//    if(autoPlan == MIDDLE_ONE_CONE_BALANCE){
	// 		log.writeLogEcho(true, "AutoSelect", "run Middle One Cone Balance");
	// 		autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime),
	// 			new DriveTrajectory(CoordType.kAbsoluteResetPoseTol, StopType.kBrake, 
	// 				((allianceSelection.getAlliance() == Alliance.Red) ? 
	// 					trajectoryCache.cache[TrajectoryType.MiddleOuterOneConeBalanceRed.value]:
	// 					trajectoryCache.cache[TrajectoryType.MiddleOuterOneConeBalanceBlue.value] ), 
	// 				driveTrain, log)
	// 		);
	//    }

	//    if(autoPlan == MIDDLE_BALANCE){
	// 	log.writeLogEcho(true, "AutoSelect", "run Middle Balance");
	// 	autonomousCommand = new SequentialCommandGroup(new WaitCommand(waitTime),
	// 		new DriveTrajectory(CoordType.kAbsoluteResetPoseTol, StopType.kBrake, 
	// 			((allianceSelection.getAlliance() == Alliance.Red) ? 
	// 				trajectoryCache.cache[TrajectoryType.CenterBalanceRed.value]:
	// 				trajectoryCache.cache[TrajectoryType.CenterBalanceBlue.value] ), 
	// 			driveTrain, log)
	// 	);
	//    }
	   

   if (autonomousCommand == null) {
			log.writeLogEcho(true, "AutoSelect", "No autocommand found");
			autonomousCommand = new WaitCommand(1);
		}

		return autonomousCommand;
	}

}
