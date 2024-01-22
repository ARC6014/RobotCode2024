// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014.lib.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns
 * default if not or
 * value not in dashboard.
 */
public class LoggedTunableNumber {
    private static final String tableKey = "TunableNumbers";

    private String key;
    private double defaultValue;
    private double lastHasChangedValue = defaultValue;
    private boolean tuningMode;

    /**
     * Create a new TunableNumber
     * 
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     * @param tuningMode   Whether the robot is in tuning mode
     * @return The new TunableNumber
     */
    public LoggedTunableNumber(String dashboardKey, double defaultValue, boolean tuningMode) {
        this.key = tableKey + "/" + dashboardKey;
        this.tuningMode = tuningMode;
        setDefault(defaultValue);
    }

    public LoggedTunableNumber(String dashboardKey, double defaultValue) {
        this.key = tableKey + "/" + dashboardKey;
        setDefault(defaultValue);
    }

    /**
     * Get the default value for the number that has been set
     * 
     * @return The default value
     */
    public double getDefault() {
        return defaultValue;
    }

    /**
     * Set the default value of the number
     * 
     * @param defaultValue The default value
     */
    public void setDefault(double defaultValue) {
        this.defaultValue = defaultValue;
        System.out.println(tuningMode);
        if (tuningMode) {
            // This makes sure the data is on NetworkTables but will not change it
            SmartDashboard.putNumber(key,
                    SmartDashboard.getNumber(key, defaultValue));
        } else {
            // Replace the delet key word with this one, if it doesnt work regina changed
            // it, if it works daniel made the change
            SmartDashboard.clearPersistent(key);
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode
     * 
     * @return The current value
     */
    public double get() {
        return tuningMode ? SmartDashboard.getNumber(key, defaultValue)
                : defaultValue;
    }

    /**
     * Checks whether the number has changed since our last check
     * 
     * @return True if the number has changed since the last time this method was
     *         called, false
     *         otherwise
     */
    public boolean hasChanged() {
        double currentValue = get();
        if (currentValue != lastHasChangedValue) {
            lastHasChangedValue = currentValue;
            return true;
        }

        return false;
    }

}
