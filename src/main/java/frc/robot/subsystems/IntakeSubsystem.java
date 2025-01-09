package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private double m_power = 0;
  private SparkFlex m_motor = new SparkFlex(Constants.k_intakeMotor, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_motor.getEncoder();
  private final double k_stallPower = -.075;
  private Timer m_stallTimer = new Timer();

  public enum IntakeType {INTAKE, OUTTAKE, STOP}

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    setBrakeMode(true);
    m_stallTimer.reset();
    m_stallTimer.start();
  }

  public void setPower(double power) {
    m_power = power;
  }

  public void intake() {
    m_power = Constants.CubeConstants.k_intakePower;
  }

  public void outtake() {
    m_power = Constants.CubeConstants.k_outtakePower;
  }

  public void stop() {
    m_power = Constants.k_isCubeMode ? 0 : k_stallPower;
  }

  public void setBrakeMode(boolean brake) {
    // m_motor.(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  // Lower power if game piece is acquired
  public boolean isIntakeStalled() {
    double speed = m_encoder.getVelocity();
    boolean isIntakeStalled = Math.abs(speed) < 1 && Math.abs(m_power) > k_stallPower;
    if (isIntakeStalled) {
      if (m_stallTimer.get() > .25) {
        return true;
      }
      return false;
    } 
    m_stallTimer.reset();
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_motor.set(m_power);
    SmartDashboard.putNumber("Intake Speed", m_encoder.getVelocity());
  }}