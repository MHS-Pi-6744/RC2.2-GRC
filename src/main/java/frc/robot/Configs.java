package frc.robot;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
  public static final class Default {
    public static final SparkMaxConfig Config = new SparkMaxConfig();
  }

  public static final class MAXSwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor =
          ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double nominalVoltage = 12.0;
      double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
      drivingConfig
          .encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(0.04, 0, 0)
          .outputRange(-1, 1)
          .feedForward
          .kV(drivingVelocityFeedForward);

      turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);

      turningConfig
          .absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0) // radians per second
          // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for
          // V1):
          .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

      turningConfig
          .closedLoop
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
    }
  }

  public static final class IntakeConfigs {
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
    public static final TalonFXSConfiguration pivotConfig = new TalonFXSConfiguration();

    static {
      // Configure basic settings of the intake motor
      intakeConfig
          .inverted(true)
          .idleMode(IdleMode.kCoast)
          .openLoopRampRate(0.5)
          .smartCurrentLimit(40);

      pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      pivotConfig
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(40))
                  .withStatorCurrentLimitEnable(true));

      var slot0Configs = pivotConfig.Slot0;
      slot0Configs.kS = 0.0;
      slot0Configs.kV = 0.0;
      slot0Configs.kA = 0.0;
      slot0Configs.kP = 1.5;
      slot0Configs.kI = 0;
      slot0Configs.kD = 0.0;

      var motionMagicConfigs = pivotConfig.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = 80;
      motionMagicConfigs.MotionMagicAcceleration = 160;
      motionMagicConfigs.MotionMagicJerk = 200;

      pivotConfig.ExternalFeedback.withRemoteCANcoder(new CANcoder(1));

      pivotConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
    }
  }
}
