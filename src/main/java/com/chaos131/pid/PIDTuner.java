// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.pid;

import java.util.function.Consumer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Helps tune a PID from the Dashboard */
public class PIDTuner {
    protected final String m_componentName;
    protected final boolean m_tuningEnabled;
    protected final Consumer<PIDFValue> m_pidfUpdater;
    private double m_p, m_i, m_d, m_f;

    public PIDTuner(
        String componentName,
        boolean tuningEnabled,
        PIDController pidController
    ) {
        this(componentName, tuningEnabled, pidController.getP(), pidController.getI(), pidController.getD(), (PIDFValue update) -> {
            pidController.setPID(update.P, update.I, update.D);
        });
    }

    public PIDTuner(
        String componentName,
        boolean tuningEnabled,
        double defaultP,
        double defaultI,
        double defaultD,
        Consumer<PIDFValue> pidfUpdater
    ) {
        this(componentName, tuningEnabled, defaultP, defaultI, defaultD, 0.0, pidfUpdater);
    }

    public PIDTuner(
        String componentName,
        boolean tuningEnabled,
        double defaultP,
        double defaultI,
        double defaultD,
        double defaultF,
        Consumer<PIDFValue> pidfUpdater
    ) {
        m_componentName = componentName;
        m_tuningEnabled = tuningEnabled;
        m_p = defaultP;
        m_i = defaultI;
        m_d = defaultD;
        m_f = defaultF;
        m_pidfUpdater = pidfUpdater;

        setDefaults(defaultP, defaultI, defaultD, defaultF);
        update();
    }

    private String getDSKey(String property) {
        return m_componentName + "/" + property;
    }

    public void tune() {
        if (!m_tuningEnabled) {
            return;
        }

        if (updateValues()) {
            update();
        }
    }

    protected void setDefaults(double defaultP, double defaultI, double defaultD, double defaultF) {
        if (m_tuningEnabled) {
            SmartDashboard.putNumber(getDSKey("P"), defaultP);
            SmartDashboard.putNumber(getDSKey("I"), defaultI);
            SmartDashboard.putNumber(getDSKey("D"), defaultD);
            SmartDashboard.putNumber(getDSKey("F"), defaultF);
        }
    }

    protected boolean updateValues() {
        double newP = SmartDashboard.getNumber(getDSKey("P"), m_p);
        double newI = SmartDashboard.getNumber(getDSKey("I"), m_i);
        double newD = SmartDashboard.getNumber(getDSKey("D"), m_d);
        double newF = SmartDashboard.getNumber(getDSKey("F"), m_f);

        boolean changes = false;
        if (newP != m_p || newI != m_i || newD != m_d || newF != m_f) {
            changes = true;
            m_p = newP;
            m_i = newI;
            m_d = newD;
            m_f = newF;
        }
        return changes;
    }

    protected PIDFValue toPIDFValue() {
        return new PIDFValue(m_p, m_i, m_d, m_f);
    }

    protected void update() {
        m_pidfUpdater.accept(toPIDFValue());
    }
}
