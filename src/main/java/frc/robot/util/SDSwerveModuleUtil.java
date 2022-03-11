package frc.robot.util;

import java.lang.reflect.Field;

import com.swervedrivespecialties.swervelib.SwerveModule;

/** Add your docs here. */
public class SDSwerveModuleUtil {
    
    public static <T> T getDriveMotor(SwerveModule module) {
        return getMotor(module, "driveController");
    }

    public static <T> T getSteerMotor(SwerveModule module) {
        return getMotor(module, "steerController");
    }

    private static <T> T getMotor(SwerveModule module, String controllerName) {
        try {
            Field field = module.getClass().getDeclaredField(controllerName);
            field.setAccessible(true);;
            Object controller = field.get(module);
            Field motorField = controller.getClass().getDeclaredField("motor");
            motorField.setAccessible(true);
            Object motorObj = motorField.get(controller);
            return (T) motorObj;
        } catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e) {
            throw new RuntimeException("Unable to reflectively access motor: " + e.getMessage(), e);
        }
    }
}
