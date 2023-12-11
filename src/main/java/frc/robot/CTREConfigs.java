package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.Constants.WristConstants;

public final class CTREConfigs {

    // Configuration for swerve drive motors
    public static final TalonFXConfiguration swerveDriveFXConfig;
    static {
        swerveDriveFXConfig = new TalonFXConfiguration();

        swerveDriveFXConfig.voltageCompSaturation = 12.0;
        swerveDriveFXConfig.neutralDeadband = 0.0;

        swerveDriveFXConfig.slot0.kP = 0.10;     // Team364 uses 0.10             // CALIBRATED
        swerveDriveFXConfig.slot0.kI = 0.0;
        swerveDriveFXConfig.slot0.kD = 0.005;                                   // CALIBRATED
        swerveDriveFXConfig.slot0.kF = 0.0;     // Use arbitrary FF instead
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = 0.0;     // Team364 uses 0.25        
        swerveDriveFXConfig.closedloopRamp = 0.0;                               

        // Supply current limit is typically used to prevent breakers from tripping.
        swerveDriveFXConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
            true, 35, 60, 0.1);

        // Stator current limit can be used to limit acceleration, torque, braking (when in brake mode), and motor heating.
        // swerveDriveFXConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(
        //     enable, currentLimit, triggerThresholdCurrent, triggerThresholdTime);
    }

    // Configuration for swerve angle motors
    public static final TalonFXConfiguration swerveAngleFXConfig;
    static {
        swerveAngleFXConfig = new TalonFXConfiguration();

        swerveAngleFXConfig.voltageCompSaturation = 12.0;
        swerveAngleFXConfig.neutralDeadband = 0.0;

        swerveAngleFXConfig.slot0.kP = 0.15;     // CALIBRATED = 0.15.  Team364 uses 0.60.
        swerveAngleFXConfig.slot0.kI = 0.0;
        swerveAngleFXConfig.slot0.kD = 3.0;     // CALIBRATED = 3.0.  Team364 uses 12.0.
        swerveAngleFXConfig.slot0.kF = 0.0;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveAngleFXConfig.openloopRamp = 0.1;
        swerveAngleFXConfig.closedloopRamp = 0.1;   // CALIBRATED = 0.1.

        swerveAngleFXConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
            true, 25, 40, 0.1);
    }

    // Configuration for swerve angle CanCoders
    public static final CANCoderConfiguration swerveCanCoderConfig;
    static {
        swerveCanCoderConfig = new CANCoderConfiguration();

        // swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }

    // Configuration for elevator motor
    public static final TalonFXConfiguration elevatorFXConfig;
    static {
        elevatorFXConfig = new TalonFXConfiguration();

        elevatorFXConfig.voltageCompSaturation = 12.0;
        elevatorFXConfig.neutralDeadband = 0.0;
        elevatorFXConfig.peakOutputForward = 1.0;       // up max output
        elevatorFXConfig.peakOutputReverse = -1.0;      // down max output
        elevatorFXConfig.openloopRamp = 0.3;        // 2019 elevator used 0.3
        elevatorFXConfig.closedloopRamp = 0.3;      // Calibrate if using Talon PID (currently not being used)

        elevatorFXConfig.slot0.kP = 0.0;     // Calibrate if using Talon PID (currently not being used)
        elevatorFXConfig.slot0.kI = 0.0;
        elevatorFXConfig.slot0.kD = 0.0;
        elevatorFXConfig.slot0.kF = 0.0;     // Use arbitrary FF instead
        elevatorFXConfig.slot0.integralZone = 0.0;
        elevatorFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        // Supply current limit is typically used to prevent breakers from tripping.
        // elevatorFXConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
        //     true, 35, 60, 0.1);

        // Stator current limit can be used to limit acceleration, torque, braking (when in brake mode), and motor heating.
        // elevatorFXConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(
        //     enable, currentLimit, triggerThresholdCurrent, triggerThresholdTime);
    }

    // Configuration for wrist motor
    public static final TalonFXConfiguration wristFXConfig;
    static {
        wristFXConfig = new TalonFXConfiguration();

        wristFXConfig.voltageCompSaturation = 12.0;
        wristFXConfig.neutralDeadband = 0.0;
        wristFXConfig.peakOutputForward = 1.0;       // forward max output
        wristFXConfig.peakOutputReverse = -1.0;      // back max output
        wristFXConfig.openloopRamp = 0.3;        // Start with trying 0.3
        wristFXConfig.closedloopRamp = 0.3;      // Start with trying 0.3

        // kP = (desired-output-1023max) / (error-in-encoder-ticks)
        //    = (desired-output-1.0max)*(1023max/1.0max) * kWristDegreesPerTick/(error-in-degrees) 
        wristFXConfig.slot0.kP = 0.03;     // CALIBRATED 0.02.  Tried 0.04 in lab, but bangs wrist when stowing.
        // kI = (desired-output-1023max) / [(time-ms) * (error-in-encoder-ticks)]
        //    = (desired-output-1.0max)*(1023max/1.0max) * (1.0sec/1000ms) * kWristDegreesPerTick / [(time-sec) * (error-in-degrees)]
        wristFXConfig.slot0.kI = 0.0;       // Example from manual = 0.0005
        // kD = (desired-output-1023max) * (time-ms) / (error-in-encoder-ticks)
        //    = (desired-output-1.0max)*(1023max/1.0max) * (1000ms/1.0sec) * kWristDegreesPerTick / (error-in-deg/sec)
        wristFXConfig.slot0.kD = 0.0;       // Example from manual = 500
        wristFXConfig.slot0.kF = 0.0;     // Use arbitrary FF instead
        wristFXConfig.slot0.integralZone = 0.0/WristConstants.kWristDegreesPerTick;      // Need to convert I-zone from degrees to encoder ticks
        // Max Iaccumulator value, in encoderTicks*milliseconds.  Max I power = kI * kIAccumMax.
        wristFXConfig.slot0.maxIntegralAccumulator = 100.0 / wristFXConfig.slot0.kI;

        wristFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        // wristFXConfig.forwardSoftLimitThreshold = 0.0;           //  Set in constructor instead, after calibrating encoder
        // wristFXConfig.forwardSoftLimitEnable = true;         
        // wristFXConfig.reverseSoftLimitThreshold = 0.0;
        // wristFXConfig.reverseSoftLimitEnable = true;

        // Supply current limit is typically used to prevent breakers from tripping.
        // wristFXConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
        //     true, 35, 60, 0.1);

        // Stator current limit can be used to limit acceleration, torque, braking (when in brake mode), and motor heating.
        // wristFXConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(
        //     enable, currentLimit, triggerThresholdCurrent, triggerThresholdTime);
    }
}