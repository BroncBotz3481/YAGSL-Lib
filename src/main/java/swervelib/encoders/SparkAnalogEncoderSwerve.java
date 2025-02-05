package swervelib.encoders;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import swervelib.motors.SparkMaxBrushedMotorSwerve;
import swervelib.motors.SparkSwerve;
import swervelib.motors.SwerveMotor;

/** SparkBase absolute encoder, attached through the data port analog pin. */
public class SparkAnalogEncoderSwerve extends SwerveAbsoluteEncoder {

  /**
   * {@link swervelib.motors.SparkMaxSwerve} or {@link swervelib.motors.SparkMaxBrushedMotorSwerve}
   * object.
   */
  private final SwerveMotor motor;
  /**
   * The {@link SparkAnalogSensor} representing the duty cycle encoder attached to the SparkMax
   * analog port.
   */
  public SparkAnalogSensor encoder;
  /** An {@link Alert} for if there is a failure configuring the encoder. */
  private Alert failureConfiguring;
  /** An {@link Alert} for if the absolute encoder does not support integrated offsets. */
  private Alert doesNotSupportIntegratedOffsets;

  /**
   * Create the {@link SparkAnalogEncoderSwerve} object as a analog sensor from the {@link SparkMax}
   * motor data port analog pin.
   *
   * @param motor Motor to create the encoder from.
   * @param maxVoltage Maximum voltage for analog input reading.
   */
  public SparkAnalogEncoderSwerve(SwerveMotor motor, double maxVoltage) {
    if (motor.getMotor() instanceof SparkBase) {
      this.motor = motor;
      encoder = ((SparkBase) motor.getMotor()).getAnalog();
      motor.setAbsoluteEncoder(this);
      motor.configureIntegratedEncoder(360 / maxVoltage);
      motor.setAbsoluteEncoder(null);
    } else {
      throw new RuntimeException("Motor given to instantiate SparkBaseEncoder is not a SparkBase");
    }
    failureConfiguring =
        new Alert("Encoders", "Failure configuring SparkBase Analog Encoder", AlertType.kWarning);
    doesNotSupportIntegratedOffsets =
        new Alert(
            "Encoders",
            "SparkBase Analog Sensors do not support integrated offsets",
            AlertType.kWarning);
  }

  /** Reset the encoder to factory defaults. */
  @Override
  public void factoryDefault() {
    // Do nothing
  }

  /** Clear sticky faults on the encoder. */
  @Override
  public void clearStickyFaults() {
    // Do nothing
  }

  /**
   * Configure the absolute encoder to read from [0, 360) per second.
   *
   * @param inverted Whether the encoder is inverted.
   */
  @Override
  public void configure(boolean inverted) {
    if (motor instanceof SparkSwerve) {
      var sparkBase = (SparkSwerve) motor;
      SparkBaseConfig cfg = sparkBase.getConfig();
      cfg.analogSensor.inverted(inverted);
      sparkBase.updateConfig(cfg);
    } else if (motor instanceof SparkMaxBrushedMotorSwerve) {
      var sparkMax = (SparkMaxBrushedMotorSwerve) motor;
      SparkMaxConfig cfg = sparkMax.getConfig();
      cfg.analogSensor.inverted(inverted);
      sparkMax.updateConfig(cfg);
    }
  }

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in degrees from [0, 360).
   */
  @Override
  public double getAbsolutePosition() {
    return encoder.getPosition();
  }

  /**
   * Get the instantiated absolute encoder Object.
   *
   * @return Absolute encoder object.
   */
  @Override
  public Object getAbsoluteEncoder() {
    return encoder;
  }

  /**
   * Sets the Absolute Encoder offset at the Encoder Level.
   *
   * @param offset the offset the Absolute Encoder uses as the zero point.
   * @return if setting Absolute Encoder Offset was successful or not.
   */
  @Override
  public boolean setAbsoluteEncoderOffset(double offset) {
    doesNotSupportIntegratedOffsets.set(true);
    return false;
  }

  /**
   * Get the velocity in degrees/sec.
   *
   * @return velocity in degrees/sec.
   */
  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }
}
