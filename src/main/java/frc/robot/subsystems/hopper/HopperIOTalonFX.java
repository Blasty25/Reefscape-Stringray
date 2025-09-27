package frc.robot.subsystems.hopper;

import static frc.robot.util.PhoenixUtil.tryUntilOk;
import static frc.robot.util.RobotMap.HopperMap.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class HopperIOTalonFX implements HopperIO {
  private final TalonFX talon;

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<Angle> position;

  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final VelocityVoltage velocityOut =
      new VelocityVoltage(0.0).withEnableFOC(false).withSlot(0);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public HopperIOTalonFX() {
    talon = new TalonFX(hopperTalon, "rio");
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = HopperConstants.current;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = HopperConstants.currentLower;
    config.CurrentLimits.SupplyCurrentLowerTime = HopperConstants.currentLowerTime;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    voltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    statorCurrent = talon.getTorqueCurrent();
    temperature = talon.getDeviceTemp();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, position, velocity, voltage, supplyCurrent, statorCurrent, temperature));
    tryUntilOk(5, () -> talon.optimizeBusUtilization(0, 1.0));
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        position, velocity, voltage, statorCurrent, supplyCurrent, temperature);
    inputs.motorVolts = voltage.getValueAsDouble();
    inputs.motorCurrent = supplyCurrent.getValueAsDouble();
    inputs.motorTemp = temperature.getValueAsDouble();
    inputs.motorVelocity = velocity.getValueAsDouble();
    inputs.motorStalled = (inputs.motorCurrent > 30) && (inputs.motorVelocity < 100);

    inputs.motorConnected =
        connectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(voltage, statorCurrent, supplyCurrent, temperature));
  }

  @Override
  public void setTrackPercent(double percent) {
    talon.setControl(dutyCycleOut.withOutput(percent));
  }

  @Override
  public void setVoltage(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setVelocity(double radPerSec) {
    talon.setControl(velocityOut.withVelocity(radPerSec));
  }

  @Override
  public void stop() {
    talon.setControl(voltageOut.withOutput(0.0));
  }

  @Override
  public void eject(double ejectValue) {
    talon.setControl(dutyCycleOut.withOutput(ejectValue));
  }
}
