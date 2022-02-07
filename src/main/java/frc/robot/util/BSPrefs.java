package frc.robot.util;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;

public class BSPrefs {
    private static final String PREFS_FILE = "/home/lvuser/deploy/bsprefs.csv";
    private static BSPrefs instance;
    private final Map<String, Double> preferences = new HashMap<String, Double>();

    public static final BSPrefs getInstance() {
        if (instance == null) {
            instance =  new BSPrefs(PREFS_FILE);
        }
        return instance; 
    }

    /**
     * Package visible for testing
     * @param filename
     */
    BSPrefs(String filename) {
        try (BufferedReader reader = new BufferedReader(new FileReader(filename))) {
            String key, value, line = null;
            while ((line = reader.readLine()) != null) {
                String[] parts = line.split(",");
                if (parts.length != 2) {
                    DriverStation.reportWarning("Unexpected preference entry: " + line, false);
                    continue;
                }
                key = parts[0].trim();
                value = parts[1].trim();

                preferences.put(key, Double.parseDouble(value));
            }
        } catch (FileNotFoundException fne) {
            DriverStation.reportError("Unable to find preference file: " + PREFS_FILE, true);
        } catch (Exception e) {
            DriverStation.reportError("Unexpected error reading preferences: " + e.getMessage(), true);
            throw new RuntimeException("Aborting due to preference file failure", e);
        }
    }

    public double getDouble(String key, double defaultValue) {
        return preferences.getOrDefault(key, defaultValue);
    }
}
