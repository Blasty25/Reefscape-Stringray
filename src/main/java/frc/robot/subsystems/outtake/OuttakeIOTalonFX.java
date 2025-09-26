// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import static frc.robot.subsystems.outtake.OuttakeConstants.rangingMode;
import static frc.robot.subsystems.outtake.OuttakeConstants.regionOfInterest;
import static frc.robot.util.PhoenixUtil.*;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.reduxrobotics.sensors.canandcolor.ProximityPeriod;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.RobotMap.OuttakeMap;

public class OuttakeIOTalonFX implements OuttakeIO {
  private final TalonFX talon;
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final Canandcolor can;
  private final CanandcolorSettings settings;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<Angle> position;

  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final VelocityVoltage velocityVoltage =
      new VelocityVoltage(0.0).withEnableFOC(false).withSlot(0);
  private final PositionVoltage positionVoltage =
      new PositionVoltage(0.0).withEnableFOC(false).withSlot(1);
  private final DutyCycleOut percentOut = new DutyCycleOut(0);

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public OuttakeIOTalonFX() {
    talon = new TalonFX(OuttakeMap.outtakeID, "rio");
    can = new Canandcolor(OuttakeMap.canandcolorID);
    can.resetFactoryDefaults();

    settings = can.getSettings();

    settings.setProximityIntegrationPeriod(ProximityPeriod.k5ms);
    settings.setAlignProximityFramesToIntegrationPeriod(true);

    can.setSettings(settings);

    config.MotorOutput.Inverted =
        OuttakeConstants.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = OuttakeConstants.current;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
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

    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        position, velocity, voltage, statorCurrent, supplyCurrent, temperature);

    inputs.motorConnected =
        connectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(voltage, statorCurrent, supplyCurrent, temperature));
    inputs.statorCurrent = statorCurrent.getValueAsDouble();
    inputs.velocityRadPerSec = velocity.getValueAsDouble();
    inputs.motorVoltage = voltage.getValueAsDouble();
    inputs.motorStalled = (inputs.statorCurrent > 30) && (inputs.velocityRadPerSec <= 100);

    inputs.rawMeasurement = can.getProximity();
    inputs.canAndColorConnected = can.isConnected();

    inputs.isCorralDetected = can.getProximity() <= 0.15; // Closer to 0 the closer the object is
  }

  /* Value from -1 to 1 */
  @Override
  public void setPercent(double percent) {
    talon.setControl(percentOut.withOutput(percent));
  }

  @Override
  public void stop() {
    talon.setControl(velocityVoltage.withVelocity(0));
  }

  @Override
  public void intake() {
    talon.setControl(voltageOut.withOutput(5.0)); // intake full speed
  }

  @Override
  public void setVelocity(double radPerSec) {
    talon.setControl(velocityVoltage.withVelocity(radPerSec));
  }

  /* Eject Corral Backwards at full speed */
  @Override
  public void eject(double eject) {
    talon.setControl(percentOut.withOutput(eject));
  }

  @Override
  public void setVoltage(double volts) {
    voltageOut.withOutput(volts);
  }

  @Override
  public void setSimState(boolean detected) {}
}
