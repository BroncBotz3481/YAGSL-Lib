package swervelib.encoders;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import swervelib.motors.SparkMaxBrushedMotorSwerve;
import swervelib.motors.SparkSwerve;
import swervelib.motors.SwerveMotor;

/** SparkBase absolute encoder, attached through the data port. */
public class SparkEncoderSwerve extends SwerveAbsoluteEncoder {

  /** The {@link AbsoluteEncoder} representing the duty cycle encoder attached to the SparkBase. */
  public final SparkAbsoluteEncoder encoder;
  /** An {@link Alert} for if there is a failure configuring the encoder. */
  protected final Alert failureConfiguring;
  /** An {@link Alert} for if there is a failure configuring the encoder offset. */
  protected final Alert offsetFailure;
  /** {@link SparkMaxBrushedMotorSwerve} or {@link SparkSwerve} instance. */
  protected final SwerveMotor motor;

  /**
   * Create the {@link SparkEncoderSwerve} object as a duty cycle from the {@link
   * com.revrobotics.spark.SparkBase} motor.
   *
   * @param motor Motor to create the encoder from.
   * @param conversionFactor The conversion factor to set if the output is not from 0 to 360.
   */
  public SparkEncoderSwerve(SwerveMotor motor, int conversionFactor) {
    failureConfiguring =
        new Alert("Encoders", "Failure configuring SparkBase Absolute Encoder", AlertType.kWarning);
    offsetFailure =
        new Alert("Encoders", "Failure to set Absolute Encoder Offset", AlertType.kWarning);
    if (motor.getMotor() instanceof SparkBase) {
      this.motor = motor;
      encoder = ((SparkBase) motor.getMotor()).getAbsoluteEncoder();
      motor.setAbsoluteEncoder(this);
      motor.configureIntegratedEncoder(conversionFactor);
      motor.setAbsoluteEncoder(null);
    } else {
      throw new RuntimeException("Motor given to instantiate SparkBaseEncoder is not a SparkBase");
    }
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
      SparkBaseConfig cfg = ((SparkSwerve) motor).getConfig();
      cfg.absoluteEncoder.inverted(inverted);
      ((SparkSwerve) motor).updateConfig(cfg);
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
   * Sets the Absolute Encoder Offset inside of the SparkBase's Memory.
   *
   * @param offset the offset the Absolute Encoder uses as the zero point.
   * @return if setting Absolute Encoder Offset was successful or not.
   */
  @Override
  public boolean setAbsoluteEncoderOffset(double offset) {
    if (motor instanceof SparkSwerve) {
      var sparkBase = (SparkSwerve) motor;
      SparkBaseConfig cfg = sparkBase.getConfig();
      cfg.absoluteEncoder.zeroOffset(offset);
      sparkBase.updateConfig(cfg);
      return true;
    } else if (motor instanceof SparkMaxBrushedMotorSwerve) {
      var sparkMax = (SparkMaxBrushedMotorSwerve) motor;
      SparkMaxConfig cfg = sparkMax.getConfig();
      cfg.absoluteEncoder.zeroOffset(offset);
      sparkMax.updateConfig(cfg);
      return true;
    }
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
