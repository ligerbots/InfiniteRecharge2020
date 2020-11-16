/*
 * Create a wrapper around the AHRS (navX) class to support simulation
 */

package frc.robot.simulation;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;

public class AHRSSimWrapper extends AHRS {
  // Need to have a simulated device, plus some places to hold the values
  private SimDevice m_simDevice;
  private SimDouble m_simAngle = null;
  private SimDouble m_simRate = null;
  private SimDouble m_simPitch = null;
 
  public AHRSSimWrapper(SPI.Port kmxp, byte update_rate_hz) {
    super(kmxp, update_rate_hz);

    // Create the SimDevice. If it returns null, we are not in simulation
    m_simDevice = SimDevice.create("AHRS", kmxp.value);
    if (m_simDevice != null) {
      m_simAngle = m_simDevice.createDouble("Angle", false, 0.0);
      m_simRate = m_simDevice.createDouble("Rate", false, 0.0);
      m_simPitch = m_simDevice.createDouble("Pitch", false, 0.0);
    }
  }

  @Override
  public double getAngle() {
    if (m_simAngle != null) {
      return m_simAngle.get();
    }
    return super.getAngle();
  }

  @Override
  public double getRate() {
    if (m_simRate != null) {
      return m_simRate.get();
    }
    return super.getRate();
  }

  @Override
  public float getPitch() {
    if (m_simPitch != null) {
      return (float)m_simPitch.get();
    }
    return super.getPitch();
  }
}
