package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Thin wrapper around the CTRE CANdle so commands can set status LEDs safely.
 * Design choices:
 * - Configure once in the constructor (brightness, strip type) to keep runtime code simple.
 * - Provide small helpers for "green" and "off" so calling code reads clearly.
 * - Expose a simple animation for future debugging (disabled by default).
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
    config.brightnessScalar = 1.0;            // Full brightness; tune down if too bright.
    config.statusLedOffWhenActive = true;     // Hide the CANdle’s own status LED when driving the strip.
    candle.configAllSettings(config, 100);

    setOff(); // Start dark so we never blind drivers at boot.
  }

  /** Set the full strip to solid green. */
  public void setGreen() {
    // setLEDs expects RGB (0-255), white (0-255), starting index, and count.
    candle.setLEDs(0, 255, 0, 0, 0, ledCount);
  }

  /** Set the full strip to solid red. */
  public void setRed() {
    candle.setLEDs(255, 0, 0, 0, 0, ledCount);
  }

  /** Set the full strip to solid yellow (red + green). */
  public void setYellow() {
    candle.setLEDs(255, 255, 0, 0, 0, ledCount);
  }

  /** Turn the strip off. */
  public void setOff() {
    candle.setLEDs(0, 0, 0, 0, 0, ledCount);
  }
}
