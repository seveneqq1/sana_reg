package org.firstinspires.ftc.teamcode.bobot.util;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class RegShooterTest {
    private boolean savedCalibrated;
    private boolean savedUseQuadratic;
    private double savedA;
    private double savedB;
    private double savedC;
    private double savedSlope;
    private double savedIntercept;

    @Before
    public void saveConfiguration() {
        savedCalibrated = RegShooter.calibrated;
        savedUseQuadratic = RegShooter.useQuadratic;
        savedA = RegShooter.quadraticA;
        savedB = RegShooter.quadraticB;
        savedC = RegShooter.quadraticC;
        savedSlope = RegShooter.slopeTPSPerInch;
        savedIntercept = RegShooter.interceptTPS;
        RegShooter.calibrated = false;
    }

    @After
    public void restoreConfiguration() {
        RegShooter.calibrated = savedCalibrated;
        RegShooter.useQuadratic = savedUseQuadratic;
        RegShooter.quadraticA = savedA;
        RegShooter.quadraticB = savedB;
        RegShooter.quadraticC = savedC;
        RegShooter.slopeTPSPerInch = savedSlope;
        RegShooter.interceptTPS = savedIntercept;
    }

    @Test
    public void usesCurrentFixedTargetUntilEquationIsCalibrated() {
        assertEquals(1400, RegShooter.velForDist(60, 1400), 0);
        assertEquals(1500, RegShooter.velForDist(90, 1500), 0);
    }

    @Test
    public void evaluatesSuppliedQuadraticFit() {
        RegShooter.calibrated = true;
        RegShooter.useQuadratic = true;
        assertEquals(1304.39453, RegShooter.velForDist(60, 1400), 1e-6);
        assertEquals(1884.92027, RegShooter.velForDist(90, 1400), 1e-6);
    }

    @Test
    public void switchesToSuppliedLinearFit() {
        RegShooter.calibrated = true;
        RegShooter.useQuadratic = false;
        assertEquals(1383.729, RegShooter.velForDist(60, 1400), 1e-6);
        assertEquals(1714.5435, RegShooter.velForDist(90, 1400), 1e-6);
    }

    @Test
    public void evaluatesEquationInInchesAndTPS() {
        RegShooter.calibrated = true;
        RegShooter.useQuadratic = false;
        RegShooter.slopeTPSPerInch = 5;
        RegShooter.interceptTPS = 1000;
        assertEquals(1300, RegShooter.velForDist(60, 1400), 0);
        assertEquals(1450, RegShooter.velForDist(90, 1400), 0);
    }

    @Test
    public void rejectsInvalidDistanceOrEquationInsteadOfCommandingSpeed() {
        RegShooter.calibrated = true;
        RegShooter.useQuadratic = false;
        RegShooter.slopeTPSPerInch = 5;
        RegShooter.interceptTPS = 1000;
        assertEquals(0, RegShooter.velForDist(Double.NaN, 1400), 0);
        assertEquals(0, RegShooter.velForDist(Double.POSITIVE_INFINITY, 1400), 0);
        assertEquals(0, RegShooter.velForDist(-1, 1400), 0);
        RegShooter.interceptTPS = -1000;
        assertEquals(0, RegShooter.velForDist(60, 1400), 0);
        RegShooter.interceptTPS = Double.NaN;
        assertEquals(0, RegShooter.velForDist(60, 1400), 0);
    }

    @Test
    public void rejectsInvalidQuadraticOutput() {
        RegShooter.calibrated = true;
        RegShooter.useQuadratic = true;
        assertEquals(0, RegShooter.velForDist(Double.MAX_VALUE, 1400), 0);
        RegShooter.quadraticA = Double.NaN;
        assertEquals(0, RegShooter.velForDist(60, 1400), 0);
    }
}
