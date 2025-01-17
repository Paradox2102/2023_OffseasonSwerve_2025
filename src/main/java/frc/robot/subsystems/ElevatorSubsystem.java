// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX m_motor = new TalonFX(Constants.k_elevatorMotor, "rio");
  private TalonFX m_follower = new TalonFX(Constants.k_elevatorFollower, "rio");
  private DigitalInput m_midSwitch = new DigitalInput(Constants.k_midSwitch);
  private DigitalInput m_bottomSwitch = new DigitalInput(Constants.k_bottomSwitch);
  private DigitalInput m_topSwitch = new DigitalInput(Constants.k_topSwitch);
  private double m_power = 0;
  private double m_targetExtentInches = 0;
  private boolean m_manualControl = false;

  private final double k_p = .04;
  private final double k_i = 0;
  private final double k_d = .004;
  private PIDController m_PID = new PIDController(k_p, k_i, k_d);

  private final double k_FLow = .01;
  private final double k_FHigh = .008;
  private final double k_midHeightInches = 11;
  private final double k_maxDownPower = -.1;
  private boolean m_brake = true;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_motor.setPosition(0);
    m_follower.setPosition(0);
    m_follower.setControl(new Follower(m_motor.getDeviceID(), true));
    setBrakeMode(true);
    m_motor.setPosition(0);
    setBrakeMode(true);
  }

  public void setExtentInches(double targetExtentInches) {
    m_targetExtentInches = targetExtentInches;
  }

  public void manualControl(boolean up, boolean manual) {
    m_manualControl = manual;
    m_targetExtentInches = getExtentInches() + 1 * (up ? 1 : -1);
    m_power = .15 * (up ? 1 : -1) + getF();
  }

  private double getF() {
    if (getExtentInches() <= 1) {
      return -.1;
    }
    return getExtentInches() > k_midHeightInches ? k_FHigh : k_FLow;
  }

  private void setPower(double power) {
    m_motor.set(-power);
  }

  public void setBrakeMode(boolean brake) {
    m_motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    m_brake = brake;
  }

  public boolean getBrake() {
    return m_brake;
  }

  private double getExtentInches() {
    return -m_motor.getPosition().getValueAsDouble() * Constants.k_elevatorTicksToInches;
  }

  private void checkLimitSwitches() {
    double extent = getExtentInches();
    if (m_power > 0) {
      if ((!m_midSwitch.get() && !m_topSwitch.get()) || extent >= Constants.k_maxExtentInches) {
        m_power = extent < k_midHeightInches ? k_FLow : k_FHigh;
        m_motor.setPosition(Constants.k_maxExtentInches * Constants.k_elevatorInchesToTicks);
      } 
    } else if (m_power < 0) {
      if (!m_bottomSwitch.get() || extent <= Constants.k_minExtentInches) {
        m_power = 0;
        m_motor.setPosition(Constants.k_minExtentInches * Constants.k_elevatorInchesToTicks);
      }
    }
  }

  private void runP() {
    double extentInches = getExtentInches();
    if (!m_manualControl) {
      m_power = m_PID.calculate(extentInches, m_targetExtentInches) + getF();
      m_power = m_power < k_maxDownPower ? k_maxDownPower : m_power;
      m_power = m_power > .4 ? .4 : m_power;
    }
    SmartDashboard.putBoolean("Bottom Switch", m_bottomSwitch.get());
    SmartDashboard.putBoolean("Top Switch", m_topSwitch.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runP();
    checkLimitSwitches();
    SmartDashboard.putNumber("Elevator Extent", getExtentInches());
    SmartDashboard.putBoolean("Bottom Switch", m_bottomSwitch.get());
    SmartDashboard.putBoolean("Mid Switch", m_midSwitch.get());
    SmartDashboard.putBoolean("High Switch", m_topSwitch.get());
    setPower(m_power);
  }
}
