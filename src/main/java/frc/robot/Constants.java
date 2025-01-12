// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double k_startAngleDegrees = 90;
  public final static double k_fieldWidthMeters = 8.0137;

  public static final double k_xFrontCameraOffsetInches = 12;
  public static final double k_yFrontCameraOffsetInches = 7.5;
  public static final double k_frontCameraAngle = 0;

  public static final double k_xBackCameraOffsetInches = -12;
  public static final double k_yBackCameraOffsetInches = -7.5;
  public static final double k_backCameraAngle = 180;

  public static boolean k_isCubeMode = true;
  public static boolean k_hasGamePiece = false;
  public static ArmPosition k_armPosition = ArmPosition.NEUTRAL;

  // Non-Drive motors
  public static final int k_intakeMotor = 10;
  public static final int k_elevatorMotor = 8;
  public static final int k_elevatorFollower = 14;
  public static final int k_wristMotor = 9;
  public static final int k_topSwitch = 7;
  public static final int k_midSwitch = 8;
  public static final int k_bottomSwitch = 9;

  // Elevator Constants
  public static final double k_minExtentInches = 0;
  public static final double k_maxExtentInches = 46;
  public static final double k_elevatorTicksToInches = 5.0/2.33;
  public static final double k_elevatorInchesToTicks = 1.0/k_elevatorTicksToInches;

  // Wrist Constants
  public static final double k_wristTicksToDegrees = 90.0/44.5;
  public static final double k_wristDegreestoTicks = 1.0/k_wristTicksToDegrees;
  public static final double k_minAngleDegrees = 0;
  public static final double k_maxAngleDegrees = 165;

  // Neutral Pose
  public static final double k_neutralElevatorInches = 2;
  public static final double k_neutralWristDegrees = 0;

  public enum ArmPosition {
    HIGH,
    MID,
    SINGLE,
    DOUBLE,
    GROUND,
    NEUTRAL
  }

  // Cube presets
  public static final class CubeConstants {
    // Intake
    public static final double k_intakePower = -.4;
    public static final double k_outtakePower = .3;
    public static final double k_intakeF = 0;

    // Elevator
    public static final double k_highElevatorInches = 39.9;
    public static final double k_midElevatorInches = 22;
    public static final double k_singleElevatorInches = 16.6;
    public static final double k_doubleElevatorInches = 40;
    public static final double k_groundElevatorInches = 2.7;

    // Wrist
    public static final double k_highWristDegrees = 46.1;
    public static final double k_midWristDegrees = 46.1;
    public static final double k_singleWristDegrees = 2.3;
    ;
    public static final double k_doubleWristDegrees = 55.7;
    public static final double k_groundWristDegrees = 87.4;
  }

  // Cone presets
  public static final class ConeConstants {
    // Intake
    public static final double k_intakePower = 1;
    public static final double k_outtakePower = -CubeConstants.k_outtakePower;
    public static final double k_intakeF = -CubeConstants.k_intakeF;

    // Elevator
    public static final double k_highElevatorInches = 44.5;
    public static final double k_midElevatorInches = 29;
    public static final double k_singleElevatorInches = 6.22;
    public static final double k_doubleElevatorInches = 16.6;
    public static final double k_groundElevatorInches = 15.4;

    // Wrist
    public static final double k_highWristDegrees = 123;
    public static final double k_midWristDegrees = 129;
    public static final double k_singleWristDegrees = 83.3;
    public static final double k_doubleWristDegrees = 2.3;
    public static final double k_groundWristDegrees = 67.5;
  }

  // DRIVETRAIN SPARK MAX IDs
  // Front of robot is opposite battery
  public static final int k_FRDriveMotor = 1; // Front Right // 3
  public static final int k_FLDriveMotor = 3; // Front Left // 1
  public static final int k_BRDriveMotor = 5; // Back Right // 7
  public static final int k_BLDriveMotor = 7; // Back Left // 5

  public static final int k_FRTurningMotor = 2; // 4
  public static final int k_FLTurningMotor = 4; // 2
  public static final int k_BRTurningMotor = 6; // 8
  public static final int k_BLTurningMotor = 8; // 6

  public static final boolean k_gyroReversed = false;

  // Angular offsets of the modules relative to the chassis in radians

  public static final double k_driveRadius = .475953574;

  // Bolt
  public static final double k_FLOffset = 0.938 - (Math.PI / 2);//2.373
  public static final double k_FROffset = 0.152;//4.708
  public static final double k_BLOffset = 5.297 + (Math.PI);//-1.314
  public static final double k_BROffset = 0.778 + (Math.PI / 2);//2.254

  public static final boolean k_isGyroReversed = true;

  public static final int k_drivingMotorPinionTeeth = 14;

  public static final double k_driveWidth = Units.inchesToMeters(26.5);
  public static final double k_driveLength = Units.inchesToMeters(26.5);
  public static final double k_wheelDiameterMeters = .0762;
  public static final double k_drivingMotorReduction = (45.0 * 22) / (k_drivingMotorPinionTeeth * 15);

  public static final double k_driveTicksToMetersVelocity = ((k_wheelDiameterMeters * Math.PI)
      / k_drivingMotorReduction) / 60.0;
  public static final double k_driveTicksToMetersPosition = (k_wheelDiameterMeters * Math.PI) / k_drivingMotorReduction;
  public static final double k_turnTicksToDegreesVelocity = (2 * Math.PI) / 60.0;
  public static final double k_turnTicksToRadiansPosition = (2 * Math.PI);

  public static final double k_turningEncoderPositionPIDMinInput = 0; // radians
  public static final double k_turningEncoderPositionPIDMaxInput = k_turnTicksToRadiansPosition; // radians

  public static final boolean k_turningEncoderInverted = true;

  public static final double k_freeSpeedRPM = 5676;
  public static final double k_drivingMotorFreeSpeedRps = k_freeSpeedRPM / 60.0;
  public static final double k_wheelCircumferenceMeters = k_wheelDiameterMeters * Math.PI;
  public static final double k_driveWheelFreeSpeedRps = (k_drivingMotorFreeSpeedRps * k_wheelCircumferenceMeters)
      / k_drivingMotorReduction;

  // Swerve Module Drive PID
  public static final double k_driveP = 0.04;
  public static final double k_driveI = 0;
  public static final double k_driveD = 0;
  public static final double k_driveFF = 1 / k_driveWheelFreeSpeedRps;
  public static final double k_drivingMinOutput = -1;
  public static final double k_drivingMaxOutput = 1;

  // Swerve Module Turn PID
  public static final double k_turnP = 1;
  public static final double k_turnI = 0;
  public static final double k_turnD = 0;
  public static final double k_turnFF = 0;
  public static final double k_turningMinOutput = -1;
  public static final double k_turningMaxOutput = 1;

  public static final int k_driveMotorCurrentLimit = 50; // amps
  public static final int k_turnMotorCurrentLimit = 20; // amps

  // Driving Constants
  public static final double k_maxSpeedMetersPerSecond = 4.8;
  public static final double k_maxDriveAcceleration = 3;
  public static final double k_maxAngularSpeed = Math.PI; // radians per second
  public static final double k_maxAngularAcceleration = Math.PI;

  public static final double k_directionSlewRate = 3; // radians per second
  public static final double k_magnitudeSlewRate = 3.25; // percent per second (1 = 100%)
  public static final double k_rotationalSlewRate = 3; // percent per second (1 = 100%)

  public static final SwerveModuleState[] k_defaultState = {
      new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
      new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4)),
      new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4)),
      new SwerveModuleState(0, new Rotation2d(Math.PI / 4))
  };

  public static final int k_LEDLength = 72;

  public static final double k_driveDeadband = 0.1;
  public static final TrapezoidProfile.Constraints k_thetaControllerConstraints = new TrapezoidProfile.Constraints(
      k_maxSpeedMetersPerSecond, k_maxAngularAcceleration);

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int k_DrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double k_DrivingMotorFreeSpeedRps = 6784 / 60;
    public static final double k_WheelDiameterMeters = 0.0762;
    public static final double k_WheelCircumferenceMeters = k_WheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double k_DrivingMotorReduction = (45.0 * 22) / (k_DrivingMotorPinionTeeth * 15);
    public static final double k_DriveWheelFreeSpeedRps = (k_DrivingMotorFreeSpeedRps * k_WheelCircumferenceMeters)
        / k_DrivingMotorReduction;
  }

  public final class MotorConfigs {
    public static final class SwerveModule {
      public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
      public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
      public static SparkMaxConfig coastDriveConfig = new SparkMaxConfig();
      public static SparkMaxConfig coastTurnConfig = new SparkMaxConfig();

      static {
        // Use module constants to calculate conversion factors and feed forward gain.
        double drivingFactor = ModuleConstants.k_WheelDiameterMeters * Math.PI
            / ModuleConstants.k_DrivingMotorReduction;
        double turningFactor = 2 * Math.PI;
        double drivingVelocityFeedForward = 1 / ModuleConstants.k_DriveWheelFreeSpeedRps;

        drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
        drivingConfig.encoder
            .positionConversionFactor(drivingFactor) // meters
            .velocityConversionFactor(drivingFactor / 60.0); // meters per second
        drivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(0.04, 0, 0)
            .velocityFF(drivingVelocityFeedForward)
            .outputRange(-1, 1);

        turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        turningConfig.absoluteEncoder
            // Invert the turning encoder, since the output shaft rotates in the opposite
            // direction of the steering motor in the MAXSwerve Module.
            .inverted(true)
            .positionConversionFactor(turningFactor) // radians
            .velocityConversionFactor(turningFactor / 60.0); // radians per second
        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(1, 0, 0)
            .outputRange(-1, 1)
            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, turningFactor);
        coastDriveConfig = drivingConfig;
        coastTurnConfig = turningConfig;
        
        coastDriveConfig.idleMode(IdleMode.kCoast);
        coastTurnConfig.idleMode(IdleMode.kCoast);
      }
    }
  }

}