package com.chaos131.util;

import java.util.Set;

import com.chaos131.logging.LogManager;
import com.chaos131.pid.PIDFValue;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose3d;

public class TalonFXWrapper extends TalonFX {
    // Configs
    protected TalonFXConfiguration m_config;
    protected DutyCycleOut m_driveDutyCycle;
    protected ControlRequest m_motorController;
    protected PositionType m_type;

    private static final PIDFValue _pidf = new PIDFValue(0.8, 0, 0.05, 0);
    private static final FullFeedForward _ff = new FullFeedForward.VelocityFeedForward(0.2, 0, 0);
    private static final VelocityVoltage _mc = new VelocityVoltage(0.0);

    // Describes the purpose of the motor
    static public enum PositionType {
        SwerveFrontLeft,
        SwerveFrontRight,
        SwerveBackLeft,
        SwerveBackRight,
        Custom
    }

    // Movement
    protected PIDFValue m_pidValues;
    protected FullFeedForward m_feedForward;
    protected Pose3d m_locationOffset;
    protected double m_maxSpeed;
    protected boolean m_geometryInitialized;
    protected double m_gearRatio;
    protected double m_wheelCircumference;

    // Audio
    protected static Orchestra _orchestra;
    protected static Set<TalonFX> _instruments;
    protected static String _lastSong;

    public TalonFXWrapper(int deviceId) {
        this(deviceId, PositionType.Custom);
    }

    public TalonFXWrapper(int deviceId, PositionType type) {
        this(deviceId, type, null, null, 0, null);
    }

    public TalonFXWrapper(int deviceId, PositionType type, PIDFValue pidf, FullFeedForward ff, double max_speed, ControlRequest motor_controller) {
        super(deviceId);
        m_type = type;

        if (pidf == null) pidf = _pidf;
        if (ff == null) ff = _ff;
        if (motor_controller == null) motor_controller = _mc;

        m_driveDutyCycle = new DutyCycleOut(0);
        m_motorController = motor_controller;
        m_maxSpeed = max_speed;
        m_feedForward = ff;

        m_config = new TalonFXConfiguration();
        m_config.Audio.BeepOnBoot = false;
        m_config.Audio.BeepOnConfig = false;
        m_config.Slot0.kP = pidf.P;
        m_config.Slot0.kI = pidf.I;
        m_config.Slot0.kD = pidf.D;
        m_config.Slot0.kV = pidf.F;
        getConfigurator().apply(m_config);

        _instruments.add(this);
        if (_orchestra == null) {
            _orchestra = new Orchestra();
        }
        _orchestra.addInstrument(this, _instruments.size()-1);
    }


    /*****************
     * State functions
     *****************/

    /**
     * 
     * @param pidf
     */
    public void updatePID(PIDFValue pidf) {
        m_config.Slot0.kP = pidf.P;
        m_config.Slot0.kI = pidf.I;
        m_config.Slot0.kD = pidf.D;
        m_config.Slot0.kV = pidf.F;
        getConfigurator().apply(m_config.Slot0);
    }

    
    /**
     * I don't think this should ever change but here it is?
     * Updates the Feed Forward values for calculation using the baseline WPILib tools.
     * They're gain coefficients for updating the feed forward target velocities.
     * They certainly look like a kinematic equation, don't they!
     * 
     * @param s - (volts) Static friction of the wheels to the ground.
     * @param v - (volts * seconds / distance) Velocity of the robot.
     * @param a - (volts * seconds^2 / distance) Acceleration of the robot.
     * @param g - (volts * seconds^2 / distance) Optional external acceleration component, typically due to gravity.
     */
    public void updateSVAG(double s, double v, double a, double g) {
        m_feedForward.updateValues(s, v, a, g);
    }
    public void updateSVA(double s, double v, double a) {
        updateSVAG(s, v, a, 0);
    }

    /**
     * 
     * @param val
     */
    public void setInvertedState(InvertedValue val) {
        m_config.MotorOutput.Inverted = val;
        getConfigurator().apply(m_config.MotorOutput);
    }

    /**
     * 
     */
    public void setNeutralMode(NeutralModeValue mode) {
        m_config.MotorOutput.NeutralMode = mode;
        getConfigurator().apply(m_config.MotorOutput);
    }

    /**
     * 
     * @param d
     */
    public void setRampUpPeriod(double d) {
        throw new UnsupportedOperationException("Not yet implemented.");
    }

    /**
     * Loads the gear ratios and wheel diameter into the state so we can properly calculate 
     * 
     * @param diameter - the diameter of the wheel, this gets converted into the circumference on the backend
     * @param offset - The location on the robot of the wheel. This is mostly useful for swerve, but can be helpful with projectile calcs.
     * This follows the standard coordinates of Front is +x and Left is +y.
     * @param gear_ratio - the ratio of the motor to the mechanism (defaults to 1).
     */
    public void setWheelGeometry(double diameter, Pose3d offset, double gear_ratio) {
        // Parameter guards
        if (diameter < 0) {
            diameter = -diameter;
        }
        boolean valid = true;
        switch (m_type) {
            case SwerveFrontLeft:
                valid = offset.getX() > 0 && offset.getY() > 0;
                break;
            case SwerveBackLeft:
                valid = offset.getX() < 0 && offset.getY() > 0;
                break;
            case SwerveBackRight:
                valid = offset.getX() < 0 && offset.getY() < 0;
                break;
            case SwerveFrontRight:
                valid = offset.getX() > 0 && offset.getY() < 0;
                break;
            default:
                break;
        };
        if (!valid) {
            throw new UnsupportedOperationException("Offsets are invalid for the given swerve type");
        }
        if (gear_ratio < 1) {
            LogManager.getInstance().addString("TalonFX Warning", true, () -> "Gear Ratio is less than 1, which is likely incorrect. Verify accuracy.");
        }

        m_wheelCircumference = Math.PI * diameter;
        m_gearRatio = gear_ratio;
        m_locationOffset = offset;

        m_config.Feedback.SensorToMechanismRatio = gear_ratio;
        StatusCode status = getConfigurator().apply(m_config.Feedback);

        if (status == StatusCode.OK) m_geometryInitialized = true;
    }
    /**
     * overloaded for convenience
     */
    public void setWheelGeometry(double diameter, Pose3d offset) {
        setWheelGeometry(diameter, offset, 1.0);
    }


    /************************
     * Movement Functionality
     ************************/

    /**
     * 
     * @param speed
     */
    public void setVelocity(double speed, boolean is_open_loop) {
        if (!m_geometryInitialized) {
            throw new UnsupportedOperationException("Geometry not initialized for setVelocity to function.");
        }

        if (is_open_loop) {
            m_driveDutyCycle.Output = speed / m_maxSpeed;
            setControl(m_driveDutyCycle);
        } else if(m_motorController instanceof VelocityVoltage ref) {
            ref.Velocity = speed / m_wheelCircumference;
            ref.FeedForward = m_feedForward.calculate(speed);
            setControl(ref);
        } else {
            throw new UnsupportedOperationException("Invalid ControlRequest data structure loaded during call to setVelocity().");
        }
    }

    /**
     * 
     */    
    public void setRPM(double rpm) {
        throw new UnsupportedOperationException("Not yet implemented.");
    }

    public StatusCode setAngularPosition(double degrees) {
        throw new UnsupportedOperationException("Not yet implemented.");
        // return setPosition(degrees);
    }


    /****************
     * Audo Functions
     ****************/

    /**
     * Plays the music on the TalonFX motor.
     * Audio is resumed if already started and paused.
     */
    public StatusCode playAudio() {
        return _orchestra.play();
    }

    /**
     * Plays a specific file from the beginning.
     * Chirp files placed in the deploy folder only need the file name and extension to be loaded.
     * 
     * @param path
     */
    public StatusCode playAudio(String path) {
        var status = prepareSong(path);
        if (status == StatusCode.OK){
            _orchestra.stop(); // may not be necessary if loadMusic() resets the index. Javadoc unclear.
            _orchestra.play();
        }
        return status;
    }

    /**
     * Stops the audio, and resets the timestamp to 0.
     * 
     * @return
     */
    public StatusCode stopAudio() {
        return _orchestra.stop();
    }

    /**
     * Loads a song into the orchestra, but does not play it.
     * @param path
     */
    public StatusCode prepareSong(String path) {
        StatusCode status = _orchestra.loadMusic(path);
        if (status == StatusCode.OK) {
            _lastSong = path;
        }
        return status;
    }
}
