package org.firstinspires.ftc.teamcode.bobot;

import org.firstinspires.ftc.teamcode.bobot.auto.comp.auto_2_stable;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;

import static org.junit.Assert.assertNotNull;

public class Auto2StableTest {
    @Test
    public void routineBuildsWithoutConflictingSubsystemRequirements() throws Exception {
        auto_2_stable opMode = new auto_2_stable();
        Field robot = auto_2_stable.class.getDeclaredField("r");
        robot.setAccessible(true);
        robot.set(opMode, RobotNew.getInstance());

        Method routine = auto_2_stable.class.getDeclaredMethod("autoRoutine");
        routine.setAccessible(true);
        assertNotNull(routine.invoke(opMode));
    }
}
