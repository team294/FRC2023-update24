// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Class to manage alliance selection and eject ball color */
public class AllianceSelection {

    private final FileLog log;
    private int logRotationKey;         // key for the logging cycle for this subsystem
    private Alliance alliance;              // Red, Blue, or Invalid

    private enum AllianceChoice {
        Red,
        Blue,
        Auto
    }
    private SendableChooser<AllianceChoice> allianceChooser = new SendableChooser<>();

    /**
     * Creates an Alliance selector.
     */
    public AllianceSelection(FileLog log) {
        this.log = log;
        logRotationKey = log.allocateLogRotation();     // Get log rotation for this subsystem

        // auto selections
		allianceChooser.setDefaultOption("Automatic", AllianceChoice.Auto);
		allianceChooser.addOption("Red", AllianceChoice.Red);
		allianceChooser.addOption("Blue", AllianceChoice.Blue);
	
		// show auto selection widget on Shuffleboard
		SmartDashboard.putData("Alliance Selection", allianceChooser);

        // Set Alliance on Shuffleboard
        setAlliance(DriverStation.getAlliance());
        log.writeLogEcho(true, "Alliance Selection", "Initialize", "Alliance from DriverStation", alliance.name());
    }

    /**
     * Sets the current alliance
     * @param alliance Red, Blue, or Invalid (= both red and blue = keep all ball without ejecting)
     */
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        log.writeLogEcho(true, "Alliance Selection", "SetAlliance", 
            "Alliance Chooser", allianceChooser.getSelected().name(), "Alliance", alliance.name());

        SmartDashboard.putBoolean("Alliance Blue", alliance != Alliance.Red);
        SmartDashboard.putBoolean("Alliance Red", alliance != Alliance.Blue);
        SmartDashboard.putString("Alliance", alliance.name());
    }

    /**
     * Returns the current alliance
     * @return Red, Blue, or Invalid (= both red and blue = keep all ball without ejecting)
     */
    public Alliance getAlliance() {
        return alliance;
    }

    /**
     * Runs once per scheduler cycle
     */
    public void periodic() {
        Alliance newAlliance = alliance;

        if (log.isMyLogRotation(logRotationKey)) {
            switch (allianceChooser.getSelected()) {
                case Auto:
                    // Do not auto change alliance in Auto or Teleop mode
                    if (DriverStation.isDisabled()) {
                        newAlliance = DriverStation.getAlliance();
                    }
                    break;
                case Red:
                    newAlliance = Alliance.Red;
                    break;
                case Blue:
                    newAlliance = Alliance.Blue;
            }

            if (alliance != newAlliance) {
                setAlliance(newAlliance);
            }

        }
    }
}
