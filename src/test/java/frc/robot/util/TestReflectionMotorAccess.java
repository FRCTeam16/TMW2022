package frc.robot.util;
import java.lang.reflect.Field;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;


/** Add your docs here. */
public class TestReflectionMotorAccess {
    SwerveModule module;

    @Before
    public void setUp() {
        module = Mk4SwerveModuleHelper.createFalcon500(
            Mk4SwerveModuleHelper.GearRatio.L2,
            0, 1, 0, -Math.toRadians(90.0));
        Assert.assertNotNull(module);
    }

    @Test
    public void testManualAccess() throws NoSuchFieldException, SecurityException, IllegalArgumentException, IllegalAccessException {        
        Field field = module.getClass().getDeclaredField("driveController");
        Assert.assertNotNull(field);
        field.setAccessible(true);;
        Object controller = field.get(module);
        Field motorField = controller.getClass().getDeclaredField("motor");
        Assert.assertNotNull(motorField);
        motorField.setAccessible(true);
        Object motorObj = motorField.get(controller);
        Assert.assertNotNull(motorObj);
        Assert.assertTrue("Not a TalonFX", motorObj instanceof TalonFX);
    }

    @Test
    public void testUtilAccess() {
        Assert.assertTrue("Not a TalonFX", SDSwerveModuleUtil.getDriveMotor(module) instanceof TalonFX);
        Assert.assertTrue("Not a TalonFX", SDSwerveModuleUtil.getSteerMotor(module) instanceof TalonFX);
    }
}
