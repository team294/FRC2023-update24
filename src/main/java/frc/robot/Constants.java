// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;


import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.TrapezoidProfileBCR;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public enum CoordType {
        kRelative,              // Relative to current robot location/facing
        kAbsolute,              // Absolute field coordinates, don't reset robot pose
        kAbsoluteResetPose,     // Absolute field coordinates, reset robot pose always
        kAbsoluteResetPoseTol;  // Absolute field coordinates, reset robot pose if robot is not close to specified position
    }

    /**
     * Options to select driving stopping types.
     */
    public enum StopType {
        kNoStop,
        kCoast,
        kBrake;
    }

    public static final class Ports{
        public static final int CANPneumaticHub = 1;

        public static final int CANDriveFrontLeftMotor = 1;
        public static final int CANDriveFrontRightMotor = 2;
        public static final int CANDriveBackLeftMotor = 3;
        public static final int CANDriveBackRightMotor = 4;

        public static final int CANDriveTurnFrontLeftMotor = 5;
        public static final int CANDriveTurnFrontRightMotor = 6;
        public static final int CANDriveTurnBackLeftMotor = 7;
        public static final int CANDriveTurnBackRightMotor = 8;

        // Note:  Remote sensors accessed by a Talon FX (Falcon 500) must have a CAN ID of 15 or less. See errata
        // in CTRE documentation "Talon FX Remote Filter Device ID Must be 15 or Less" for more details.
        // This applies to the turn encoders, which are used as remote sensors for the turn motors.
        public static final int CANTurnEncoderFrontLeft = 9;
        public static final int CANTurnEncoderFrontRight = 10;
        public static final int CANTurnEncoderBackLeft = 11;
        public static final int CANTurnEncoderBackRight = 12;

        public static final int CANElevatorMotor = 21;
        // public static final int CANElevatorMotor2 = 22;
        public static final int CANWristMotor = 45; 

        public static final int CANIntake1 = 40;
        public static final int CANIntake2 = 15;
        public static final int CANManipulator = 43; 
        public static final int CANConveyor = 47;

        // Digital IO ports
        public static final int DIOWristRevThroughBoreEncoder = 0;
        public static final int DIOManipulatorCubeSensor = 1;
        public static final int DIOManipulatorConeSensor = 2;

        // PWM ports
        public static final int PWMLEDStripTop = 0;         // LED Strip on top of robot

        // I2C ports
        // public static final int I2CcolorSensor = 0x52;       // According to REV docs, color sensor is at 0x52 = 82.  Rob had 39?

        // Pneumatic solenoid ports
        public static final int SolManipulatorFwd = 1;
        public static final int SolManipulatorRev = 0;
        public static final int SolIntakeLeftFwd = 3;       
        public static final int SolIntakeLeftRev = 2;     
        public static final int SolIntakeRightFwd = 9;       
        public static final int SolIntakeRightRev = 10;     

    }

    public static final class OIConstants {
        public static final int usbXboxController = 0;
        public static final int usbLeftJoystick = 1;
        public static final int usbRightJoystick = 2;
        public static final int usbCoPanel = 3;

        public static final double joystickDeadband = 0.01;
        public static final double manualElevatorDeadband = 0.1;
        public static final double manualWristDeadband = 0.1;
    }

    public static final class RobotDimensions {
        //left to right distance between the drivetrain wheels; should be measured from center to center
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.58721;      // CALIBRATED-3 = 0.58721 (based on robot rotating in place).  CAD geometry = 0.57785.
        //front-back distance between the drivetrain wheels; should be measured from center to center
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.58721;       // CALIBRATED-3 = 0.58721 (based on robot rotating in place).  CAD geometry = 0.57785.

    }

    public static final class SwerveConstants {
        // Encoder calibration to meters travelled or wheel facing degrees
        public static final double kEncoderCPR = 2048.0;                // CALIBRATED-3 = 2048.  Encoder counts per revolution of FalconFX motor pinion gear
        public static final double kDriveGearRatio = (6.75 / 1.0);      // CALIBRATED-3 = 6.75/1.0.  Team364 (MK3i?) = 6.86:1.  Mk4i = 8.14:1 (L1-std gears).  Mk4i = 6.75:1 (L2-fast gears)
        public static final double kTurningGearRatio = (150.0/7.0 / 1.0); // CALIBRATED-3 = 150.0/7.0.  Team364 (MK3i?) = 12.8:1.  Mk4i = 150/7 : 1
        public static final double kWheelDiameterMeters = 0.09712;        // CALIBRATED-3 = 0.09712.  Depends a little on the tread wear!
        public static final double kDriveEncoderMetersPerTick = (kWheelDiameterMeters * Math.PI) / kEncoderCPR / kDriveGearRatio;
        public static final double kTurningEncoderDegreesPerTick = 360.0/kEncoderCPR / kTurningGearRatio;
        
        // Robot calibration for feed-forward and max speeds
        // Max speed is used to keep each motor from maxing out, which preserves ratio between motors 
        // and ensures that the robot travels in the requested direction.  So, use min value of all 4 motors,
        // and further derate (initial test by 5%) to account for some battery droop under heavy loads.
        // Max speed measured values 2/12/2023:  All 4 motors are between 4.6 an 4.7 meters/sec.  So use 4.5 as a conservative value
        public static final double kMaxSpeedMetersPerSecond = 4.5;          // CALIBRATED-3
        public static final double kFullSpeedMetersPerSecond = 0.95*kMaxSpeedMetersPerSecond;
        public static final double kNominalSpeedMetersPerSecond = 0.5*kMaxSpeedMetersPerSecond;
        // Max acceleration measured values 1/13/2023: FL = 28.073, FR = 26.343, BL = 18.482, BR = 19.289
        // Max acceleration measured 1/25/2023 (with ~80lbs on robot):  Average of 4 wheels = 10.0 m/sec^2
        // Max acceleration measured 2/12/2023 (with new drive gears):  Average ~11 m/sec^2.  Keep value at 10.0 for now.
        public static final double kMaxAccelerationMetersPerSecondSquare = 10; // CALIBRATED-3
        public static final double kFullAccelerationMetersPerSecondSquare = 0.9*kMaxAccelerationMetersPerSecondSquare;
        public static final double kNominalAccelerationMetersPerSecondSquare = 3.5; // was 7.0 for week 1
        public static final double kMaxRetractingAccelerationMetersPerSecondSquare = 2; // 
        public static final double kMaxTurningRadiansPerSecond = 11.0;   // CALIBRATED-3 took 633 degreesPerSecond and converted to radians and rounded down
        public static final double kNominalTurningRadiansPerSecond = Math.PI;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 35.0;            // CALIBRATED-3 37.4 rad/sec^2
        public static final double kNominalAngularAccelerationRadiansPerSecondSquared = Math.PI;
        public static final double kVDrive = 0.2034; // CALIBRATED-3 = 0.2511.  in % output per meters per second.  Calibration says 0.2511, but better match on a trapezoid is 
        public static final double kADrive = 0.0;
        public static final double kADriveToPose = 0.060;
        public static final double kSDrive = 0.016; // CALIBRATED-3 = 0.016.  in % output

    }

      public static final class DriveConstants {
        // The locations of the wheels relative to the physical center of the robot, in meters.
        // X: + = forward.  Y: + = to the left
        // The order in which you pass in the wheel locations is the same order that
        // you will receive the module states when performing inverse kinematics. It is also expected that
        // you pass in the module states in the same order when calling the forward kinematics methods.
        // 0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight
        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                new Translation2d(RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, -RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                new Translation2d(-RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                new Translation2d(-RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, -RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2));

        // Update the offset angles in RobotPreferences (in Shuffleboard), not in this code!
        // After updating in RobotPreferences, you will need to re-start the robot code for the changes to take effect.
        // When calibrating offset, set the wheels to zero degrees with the bevel gear facing to the right
        public static double offsetAngleFrontLeftMotor = 0; // 92.3
        public static double offsetAngleFrontRightMotor = 0; // -12.8
        public static double offsetAngleBackLeftMotor = 0; // -107.6
        public static double offsetAngleBackRightMotor = 0; // -170.2

        // Driving constants to cap acceleration
        public static final double maxAccelerationRate = 10.0; //5.0          // m/s^2
        public static final double maxAccelerationRateY = 5.0; //5.0          // m/s^2
        public static final double maxAccelerationRateAtScoreMid = 7; //3.5          // m/s^2
        public static final double maxAccelerationRateBetweenScoreMidAndHigh = 6.0; //3.0          // m/s^2
        public static final double maxAccelerationRateWithElevatorUp = 1.5; //2.0          // m/s^2
        public static final double maxRotationRateWithElevatorUp = 0.8;     // rad/sec

        // Auto balance constants
        public static final double maxPitchBalancedDegrees = 5.0;       // If abs(Pitch) is under this value, then assume we are balanced
        public static final double kPDriveBalance = 0.018;       // 0.018 works, trying faster.  0.027 oscillates
        public static final double kDDriveBalance = 0.008;       // 0.018 works, trying faster.  0.027 oscillates

        public static final double kPJoystickThetaController = 3; // Theta kp value for joystick in rad/sec
      }

      public static final class TrajectoryConstants {
        public static final double ChargeStationVelocity = 1.2;
        public static final double ChargeStationBalanceVelocity = 0.2;

        // Max error for robot rotation
        public static final double maxThetaErrorDegrees = 1.0;
        public static final double maxPositionErrorMeters = 0.04; // 1.6 inches

        // Max error for interim positions (not final)
        public static final double interimThetaErrorDegrees = 2.0;        
        public static final double interimPositionErrorMeters = 0.20; // 8 inches

        // Feedback terms for holonomic drive controllers

        // X-velocity controller:  kp.  Units = (meters/sec of velocity) / (meters of position error)
        public static final double kPXController = 1;

        // Y-velocity controller:  kp.  Units = (meters/sec of velocity) / (meters of position error)  
        public static final double kPYController = 1; 

        public static final double kPThetaController = 3;   // Theta-velocity controller:  kp.  Units = (rad/sec of velocity) / (radians of angle error)

        public static final TrajectoryConfig swerveTrajectoryConfig =
            new TrajectoryConfig(
                    SwerveConstants.kNominalSpeedMetersPerSecond,
                    SwerveConstants.kNominalAccelerationMetersPerSecondSquare)
                .setKinematics(DriveConstants.kDriveKinematics);

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            SwerveConstants.kNominalTurningRadiansPerSecond, SwerveConstants.kNominalAngularAccelerationRadiansPerSecondSquared);

        /* Constraint for the DriveToPose motion profile for distance being travelled */
        public static final TrapezoidProfileBCR.Constraints kDriveProfileConstraints =
        new TrapezoidProfileBCR.Constraints(
            SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare);
      }

    public static class FieldConstants {
        public static final double length = Units.feetToMeters(54);
        public static final double width = Units.feetToMeters(27);
    }

    public static class VisionConstants {

        public static final Transform3d robotToCam =
                new Transform3d(
                    // new Translation3d(Units.inchesToMeters(6.0), 0.0, Units.inchesToMeters(30.5)),       Changed in B3
                    new Translation3d(Units.inchesToMeters(7.75), -0.005, Units.inchesToMeters(30.5)),
                    new Rotation3d(0, Units.degreesToRadians(15), 0)); // Cam mounted facing forward in center of robot
        public static final String cameraName = "CenterCamera";
        public static final double targetSideLength = Units.inchesToMeters(6);
    }

    public static final class WristConstants {
        public static final double kEncoderCPR = 2048.0;                // CALIBRATED = 2048.  Encoder counts per revolution of FalconFX motor pinion gear
        public static final double kWristGearRatio = (50.0 / 1.0);       // From CAD, should be 50:1.  Gear reduction ratio between Falcon and gear driving the wrist (planetary and chain gears)
        public static final double kWristDegreesPerTick =  360.0 / kEncoderCPR / kWristGearRatio * 0.9726;      // CALIBRATED (fudge factor 0.9726)

        public static final double maxUncalibratedPercentOutput = 0.05;     // CALIBRATED
        public static final double maxPercentOutput = 0.1;          // CALIBRATED

        // Update the REV through bore encoder offset angle in RobotPreferences (in Shuffleboard), not in this code!
        // After updating in RobotPreferences, you will need to re-start the robot code for the changes to take effect.
        // When calibrating offset, 0 deg should be with the CG of the wrist horizontal facing away from the robot,
        // and -90 deg is with the CG of the wrist resting downward.
        public static double revEncoderOffsetAngleWrist = 0;    // -49.0 deg (was 69.0 deg before changing wrist chain)

        public static final double kG = 0.03;   // CALIBRATED 0.02.  Feed foward percent-out to add to hold arm horizontal (0 deg)

        // Wrist regions
        public enum WristRegion {
            // backFar,        // In the wrist backFar region, the elevator must be in the bottom region (not allowed to go to elevator main or low regions).
            // backMid,        // In the wrist backMid region, the elevator may be in any elevator region.
            // down,           // Wrist pointed down, the elevator must be in the main region.
            back,           // In the wrist back region, the elevator must be in the bottom region (not allowed to go to elevator main).
            main,           // In the wrist main region, the elevator may be in any elevator region.
            uncalibrated    // Unknown region, wrist is not calibrated
        } 
        // Wrist region boundaries
        // public static final double boundBackFarMid = -119.0;      // Boundary between backFar and backMid regions.  CALIBRATED
        // public static final double boundBackMidDown = -116.0;      // Boundary between backMid and down regions.  CALIBRATED
        // public static final double boundDownMain = -91.0;      // Boundary between down and main regions.  CALIBRATED
        // public static final double boundDownMidpoint = (boundBackMidDown+boundDownMain)/2.0;      // Midpoint in down region
        public static final double boundBackMain = -120.0;      // Boundary between back and main regions.  CALIBRATED

        // Wrist pre-defined angles (in degrees)
        // 0 degrees = horizontal (in front of robot) relative to wrist center of gravity
        // -90 degrees = vertical = wrist is hanging "down" naturally due to gravity
        public enum WristAngle {
            lowerLimit(-138.0),      // CALIBRATED
            loadIntake(-135.0),    // Conveyor gone so this is unnecesary currently, was -117.5
            startConfig(-115.0),     // CALIBRATED
            loadHumanStation(10.0),      // CALIBRATED
            scoreLow(0.0),
            scoreMidHigh(20.0),         // Was 10.0
            elevatorMoving(32.0),    // CALIBRATED
            upperLimit(32.0);       // CALIBRATED
            // score low 5 inches
            @SuppressWarnings({"MemberName", "PMD.SingularField"})
            public final double value;
            WristAngle(double value) { this.value = value; }
        }
      }

      public static final class ElevatorConstants {
        public static final double kEncoderCPR = 2048.0;                // CALIBRATED = 2048.  Encoder counts per revolution of FalconFX motor pinion gear
        public static final double kElevGearRatio = (12.0 / 1.0);        // CALIBRATED.  Gear reduction ratio between Falcon and gear driving the elevator
        public static final double kElevStages = 2.0;                   // Upper stage moves 2x compared to lower stage
        public static final double kElevGearDiameterInches = 1.273;       // CALIBRATED.  Diameter of the gear driving the elevator in inches.  Per CAD = 1.273.  Calibrated = 1.276.
        public static final double kElevEncoderInchesPerTick = (kElevGearDiameterInches * Math.PI) / kEncoderCPR / kElevGearRatio * kElevStages;

        public static final double maxUncalibratedPercentOutput = 0.10;     // CALIBRATED
        public static final double maxManualPercentOutput = 0.50;  // CALIBRATED

        // Elevator regions
        public enum ElevatorRegion {
            bottom,     // In the elevator bottom region, the wrist may be in any wrist region.
            // low,        // Slightly up, wrist can not go far-back or main
            main,       // In the elevator main region, the wrist must be in the wrist main region (not allowed to go to wrist back region).
            uncalibrated;       // Unknown region, elevator is not calibrated.
        }
        // Elevator region boundaries
        // public static final double boundBottomLow = 2.0;        // Boundary between bottom and low regions
        // public static final double boundMainLow = 2.0;      // Boundary between low and main regions
        public static final double boundBottomMain = 2.5;      // Boundary between low and main regions

        // Elevator pre-defined positions (in inches from bottom of elevator)
        public enum ElevatorPosition {
            lowerLimit(0.0),        // CALIBRATED
            bottom(0.0),            // CALIBRATED
            belowScoringPegs(3.0),
            loadingStationCube(35.0),   // CALIBRATED
            loadingStationCone(45.0),   // CALIBRATED
            scoreLow(0.0),          // CALIBRATED, was 5.0
            scoreMidCone(21.0),     // CALIBRATED
            scoreHighCone(41.0),    // CALIBRATED
            upperLimit(45.4);       // CALIBRATED
            // score low 5 inches
            @SuppressWarnings({"MemberName", "PMD.SingularField"})
            public final double value;
            ElevatorPosition(double value) { this.value = value; }
        }

        public enum ElevatorSlewRegion {
            min(6.0, 4.5, 11.0),
            low(15.0, 3.0, 5.0),
            medium(30.0, 2.2, 1.6),
            max(45.4, 1.0, 0.8);

            public final double position, velocity, rotationRate;
            /**
             * 
             * @param position position of elevator (inches)
             * @param velocity max velocity of the robot when the elevator is at the position (m/s^2)
             * @param rotationRate max rotation rate of the robot when the elevator is at the position (rad/s^2)
             */
            ElevatorSlewRegion(double position, double velocity, double rotationRate) { 
                this.position = position;
                this.velocity = velocity;
                this.rotationRate = rotationRate;
            }
        }
      }

      public static final class ManipulatorConstants {
        public static final double pieceGrabPct = 1.0;  //was 0.8, changed for H5                // Motor pct when grabbing from field
        public static final double pieceGrabFromIntakePct = 1.0;        // Motor pct when grabbing from intake
        public static final double pieceHoldPct = 0.2;
      }

      public static final class IntakeConstants {
        public static final double motor1NominalIntakePct = 0.91;        // Was 0.4, occasionally popping out of top.
        public static final double motor2NominalIntakePct = 0.91;       // Was 0.52, occasionally popping out of top.
      }
}
