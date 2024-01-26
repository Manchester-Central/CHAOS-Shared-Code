package com.chaos131.util;

import java.util.Set;

import com.chaos131.logging.LogManager;
import com.chaos131.pid.PIDFValue;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;

public class TalonFXWrapper extends TalonFX {
	// Configs
	protected TalonFXConfiguration m_config;
	protected Slot0Configs m_slot0Configs;
	protected DutyCycleOut m_driveDutyCycle;
	protected ControlRequest m_motorController;


	// Movement
	protected PIDFValue m_pidValues;
	protected FullFeedForward m_feedForward;
	protected Translation2d m_locationOffset;
	protected double m_maxSpeed;
	protected boolean m_geometryInitialized;
	protected double m_gearRatio;
	protected double m_wheelCircumference;

	// Audio
	protected static Orchestra _orchestra;
	protected static Set<TalonFX> _instruments;
	protected static String _lastSong;

	public TalonFXWrapper(int deviceId, PIDFValue pidf, FullFeedForward ff, double max_speed, ControlRequest motor_controller) {
		super(deviceId);

		m_driveDutyCycle = new DutyCycleOut(0);
		m_motorController = motor_controller;
		m_maxSpeed = max_speed;
		m_feedForward = ff;

		m_config = new TalonFXConfiguration();
		m_config.Audio.BeepOnBoot = false;
		m_config.Audio.BeepOnConfig = false;
		getConfigurator().apply(m_config);

		m_slot0Configs = new Slot0Configs();
		m_slot0Configs.kP = pidf.P;
		m_slot0Configs.kI = pidf.I;
		m_slot0Configs.kD = pidf.D;
		m_slot0Configs.kV = pidf.F;
		getConfigurator().apply(m_slot0Configs);

		_instruments.add(this);
		if (_orchestra == null) {
			_orchestra = new Orchestra();
		}
		// TODO: Test this.
		// I *think* the tracks start at index 0.
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
		m_slot0Configs.kP = pidf.P;
		m_slot0Configs.kI = pidf.I;
		m_slot0Configs.kD = pidf.D;
		m_slot0Configs.kV = pidf.F;
		getConfigurator().apply(m_slot0Configs);
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
	 */
	public void updateSVA(double s, double v, double a) {
		m_feedForward = new FullFeedForward(s, 0, v, a);
	}

	/**
	 * 
	 * @param val
	 */
	public void setInvertedState(InvertedValue val) {
		m_config.MotorOutput.Inverted = val;
		getConfigurator().apply(m_config);
	}

	/**
	 * 
	 */
	public void setNeutralMode(NeutralModeValue mode) {
		m_config.MotorOutput.NeutralMode = mode;
		getConfigurator().apply(m_config);
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
	 * @param gear_ratio - the ratio of the motor to the mechanism (defaults to 1).
	 */
	public void setWheelGeometry(double diameter, double gear_ratio) {
		m_wheelCircumference = Math.PI * diameter;
		if (gear_ratio < 1) {
			LogManager.getInstance().addString("TalonFX Warning", true, () -> "Gear Ratio is less than 1, which is likely incorrect. Verify accuracy.");
		}
		m_gearRatio = gear_ratio;

		m_config.Feedback.SensorToMechanismRatio = gear_ratio;
		getConfigurator().apply(m_config);

		m_geometryInitialized = true;
	}
	/**
	 * overloaded for convenience
	 */
	public void setWheelGeometry(double diameter) {
		setWheelGeometry(diameter, 1.0);
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
			throw new UnsupportedOperationException("Invalid ControlRequest data structure loaded during call to setVelocity.");
		}
	}

	/**
	 * 
	 */
	public void setRPM() {
		throw new UnsupportedOperationException("Not yet implemented.");
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
