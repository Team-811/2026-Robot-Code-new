package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Thin wrapper around the CTRE CANdle so commands can set status LEDs safely.
 * Updated to Phoenix 6 CANdle APIs (2026 season).
 */
public class CandleLED extends SubsystemBase {
  private final CANdle candle;
  private final int ledCount;

  /**
   * @param canId CAN ID of the CANdle (from Constants).
   * @param ledCount Number of addressable LEDs on the attached strip.
   */
  public CandleLED(int canId, int ledCount) {
    this.ledCount = ledCount;
    // Use the default CAN bus ("rio") unless you wired a second bus.
    candle = new CANdle(canId, "rio");

    // Configure once up front so later calls can just set colors.
    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.BrightnessScalar = 1.0; // Full brightness; tune down if too bright.
    config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
    candle.getConfigurator().apply(config, 0.1);

    setOff(); // Start dark so we never blind drivers at boot.
  }

  private void setColor(RGBWColor color) {
    candle.setControl(new SolidColor(0, ledCount).withColor(color));
  }

  /** Set the full strip to solid green. */
  public void setGreen() {
    setColor(new RGBWColor(0, 255, 0));
  }

  /** Set the full strip to solid red. */
  public void setRed() {
    setColor(new RGBWColor(255, 0, 0));
  }

  /** Set the full strip to solid yellow (red + green). */
  public void setYellow() {
    setColor(new RGBWColor(255, 255, 0));
  }

  /** Turn the strip off. */
  public void setOff() {
    setColor(new RGBWColor(0, 0, 0));
  }
}
