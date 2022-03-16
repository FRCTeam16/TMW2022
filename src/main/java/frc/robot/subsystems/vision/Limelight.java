package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** 
 * Limelight operation support 
 * 
 * The limelight will update its network table entries at 100Mhz.
 */
public class Limelight {

    private final NetworkTable dataTable;

    public Limelight() {
        this.dataTable = NetworkTableInstance.getDefault().getTable("limelight");
        setCameraMode(CameraMode.ImageProcessing);
        setLEDMode(LEDMode.CurrentPipeline);
    }

    /**
     * Returns Scene Information from Limelight.
     * @return SceneInfo structure containing data
     */
    public SceneInfo getScene() {
        SceneInfo si = new SceneInfo();
        si.hasTarget = (dataTable.getEntry("tv").getNumber(0.0).doubleValue() > 0.5);
        si.xOffset = dataTable.getEntry("tx").getDouble(0.0);
        si.yOffset = dataTable.getEntry("ty").getDouble(0.0);
        si.targetArea = dataTable.getEntry("ta").getDouble(0.0);

        SmartDashboard.putBoolean("SceneInfo.hasTarget", si.hasTarget);
        SmartDashboard.putNumber("SceneInfo.xOffset", si.xOffset);

        // Optional items
        si.skew = dataTable.getEntry("ts").getDouble(0.0);
        si.latency = dataTable.getEntry("tl").getDouble(0.0) + 11;
        return si;
    }

    public int getCurrentPipelin() {
        return (int) dataTable.getEntry("pipeline").getNumber(-1);
    }

    public void setCurrentPipeline(int pipeline) {
        if (pipeline < 0 || pipeline > 9) {
            throw new IllegalArgumentException("Valid pipeline numbers are from 0-9, you passed in" + pipeline);
        }
        dataTable.getEntry("pipeline").setNumber(pipeline);
    }

    public CameraMode getCameraMode() {
        int rawMode = (int) dataTable.getEntry("camMode").getNumber(CameraMode.Unknown.mode);
        for (CameraMode cameraMode : CameraMode.values()) {
            if (cameraMode.mode == rawMode) {
                return cameraMode;
            }
        }
        return CameraMode.Unknown;
    }

    public void setCameraMode(CameraMode cameraMode) {
        dataTable.getEntry("camMode").setNumber(cameraMode.mode);
    }

    public LEDMode getLEDMode() {
        int rawMode = (int) dataTable.getEntry("limelight").getNumber(LEDMode.CurrentPipeline.mode);
        for (LEDMode ledMode : LEDMode.values()) {
            if (ledMode.mode == rawMode) {
                return ledMode;
            }
        }
        // Warning?
        return LEDMode.CurrentPipeline;
    }

    public void setLEDMode(LEDMode ledMode) {
        System.out.println("Setting LED Mode: " + ledMode + " | " + ledMode.mode);
        dataTable.getEntry("limelight").setNumber(ledMode.mode);
    }

    /**
     * Camera Modes
     */
    public enum CameraMode {
        Unknown(-1),
        ImageProcessing(0),
        DriverCamera(1);

        private final int mode;

        CameraMode(int mode) {
            this.mode = mode;
        }
    }

    /**
     * LED modes
     */
    public enum LEDMode {
        CurrentPipeline(0),
        ForceOff(1),
        ForceBlink(2),
        ForceOn(3);

        private final int mode;

        LEDMode(int mode) {
            this.mode = mode;
        }
    }

    /**
     * Information about the scene 
     */
    public static class SceneInfo {
        /* Whether the limelight has any valid targets */
        public boolean hasTarget = false;
        /* Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees) */
        public double xOffset = 0.0;
        /* Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees) */
        public double yOffset = 0.0;
        /* Target Area (0% of image to 100% of image) */
        public double targetArea = 0.0;

        public double skew = 0.0;
        public double latency = 0.0;
    }
}
