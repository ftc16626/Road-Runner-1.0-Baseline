package org.firstinspires.ftc.teamcode.ComponentSubClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Arrays;

/**
 * ColorSensorSubsystem
 * - Handles up to six NormalizedColorSensor instances (sensor names passed in constructor).
 * - Provides normalized -> 0..255 RGB conversion, simple "Green vs Purple" classification,
 *   aggregated readings, and utilities to enable/disable light & set gain.
 *
 * Usage:
 *   ColorSensorSubsystem cs = new ColorSensorSubsystem(hardwareMap,
 *      "first","second","third","fourth","fifth","sixth");
 *
 */
public class ColorSensorSubsystem {
    private final NormalizedColorSensor[] sensors;
    private final int sensorCount;

    public ColorSensorSubsystem(HardwareMap hardwareMap, String name0, String name1, String name2,
                                String name3, String name4, String name5) {
        String[] names = new String[]{ name0, name1, name2, name3, name4, name5 };
        sensors = new NormalizedColorSensor[6];
        int count = 0;
        for (int i = 0; i < 6; i++) {
            String nm = names[i];
            if (nm != null && nm.length() > 0) {
                try {
                    sensors[i] = hardwareMap.get(NormalizedColorSensor.class, nm);
                    count++;
                } catch (Exception e) {
                    sensors[i] = null;
                    RobotLog.ee("ColorSensorSubsystem", "missing color sensor '%s' : %s", nm, e.toString());
                }
            } else {
                sensors[i] = null;
            }
        }
        this.sensorCount = count;
    }

    /** Return number of sensors that were found in the hardware map (0..6). */
    public int getFoundSensorCount() { return sensorCount; }

    /** Enable/disable the onboard light for all sensors that are SwitchableLight. */
    public void enableLights(boolean on) {
        for (NormalizedColorSensor s : sensors) {
            if (s instanceof SwitchableLight) {
                ((SwitchableLight)s).enableLight(on);
            }
        }
    }

    /** Set sensor gain for all sensors that support it. Gain is typically around 1.0. */
    public void setGain(double gain) {
        for (NormalizedColorSensor s : sensors) {
            if (s != null) {
                try {
                    s.setGain((float) gain);
                } catch (Exception e) {
                    // Some drivers might not support setGain; ignore safely.
                }
            }
        }
    }

    /** Get NormalizedRGBA from a sensor index (0..5). Returns null if sensor missing. */
    public NormalizedRGBA getNormalizedColors(int index) {
        if (index < 0 || index >= sensors.length) return null;
        NormalizedColorSensor s = sensors[index];
        if (s == null) return null;
        return s.getNormalizedColors();
    }

    /**
     * Convert NormalizedRGBA to 0..255 int RGB array [r,g,b].
     * Uses clipping: values <0 -> 0, >1 -> 1, then multiplies by 255 and rounds.
     * Returns null if sensor missing.
     */
    public int[] getRGB(int index) {
        NormalizedRGBA n = getNormalizedColors(index);
        if (n == null) return null;
        // Some drivers provide values >1 depending on gain — clamp first.
        float r = clamp(n.red, 0f, 1f);
        float g = clamp(n.green, 0f, 1f);
        float b = clamp(n.blue, 0f, 1f);
        int Ri = Math.round(r * 255f);
        int Gi = Math.round(g * 255f);
        int Bi = Math.round(b * 255f);
        return new int[]{ Ri, Gi, Bi };
    }

    /** Helper clamp */
    private float clamp(float v, float min, float max) {
        if (v < min) return min;
        if (v > max) return max;
        return v;
    }

    /**
     * Simple classification using your heuristic:
     * - If green > blue -> "Green"
     * - else -> "Purple"
     *
     * Returns null if sensor missing.
     */
    public String getColorName(int index) {
        NormalizedRGBA n = getNormalizedColors(index);
        if (n == null) return null;
        if (n.green > n.blue) return "Green";
        return "Purple";
    }

    /** Convenience boolean: is sensor reporting 'Green' (by same rule). */
    public boolean isGreen(int index) {
        NormalizedRGBA n = getNormalizedColors(index);
        if (n == null) return false;
        return n.green > n.blue;
    }

    /** Convenience boolean for 'Purple' (not green). */
    public boolean isPurple(int index) {
        return !isGreen(index);
    }

    /** Read all sensors and return an array of color names (String or null for missing). */
    public String[] readAllColorNames() {
        String[] out = new String[sensors.length];
        for (int i = 0; i < sensors.length; i++) {
            out[i] = getColorName(i);
        }
        return out;
    }

    /** Read all RGBs as int[6][3]; missing sensors return null rows. */
    public int[][] readAllRGBs() {
        int[][] all = new int[6][];
        for (int i = 0; i < 6; i++) all[i] = getRGB(i);
        return all;
    }

    /** Returns a simple aggregated 'majority' color among available sensors ("Green","Purple", or "Unknown") */
    public String getMajorityColor() {
        int greenCount = 0, purpleCount = 0;
        for (int i = 0; i < sensors.length; i++) {
            NormalizedRGBA n = getNormalizedColors(i);
            if (n == null) continue;
            if (n.green > n.blue) greenCount++;
            else purpleCount++;
        }
        if (greenCount == 0 && purpleCount == 0) return "Unknown";
        return (greenCount >= purpleCount) ? "Green" : "Purple";
    }

    /** Debug-friendly string of all sensor readings. */
    public String debugString() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < sensors.length; i++) {
            NormalizedRGBA n = getNormalizedColors(i);
            if (n == null) {
                sb.append(String.format("S%d: (missing)\n", i+1));
            } else {
                int[] rgb = getRGB(i);
                sb.append(String.format("S%d: R=%d G=%d B=%d -> %s\n", i+1, rgb[0], rgb[1], rgb[2], getColorName(i)));
            }
        }
        return sb.toString();
    }
}
