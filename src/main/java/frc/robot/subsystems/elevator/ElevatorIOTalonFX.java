// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO {

  private final TalonFX left;
  private final TalonFX right;

  private final TalonFXConfiguration config;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temperature;

  private final StatusSignal<Voltage> followerVoltage;
  private final StatusSignal<Current> followerStatorCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Temperature> followerTemperature;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(false);
  private final MotionMagicVoltage positionTorque = new MotionMagicVoltage(0.0).withSlot(0);

  private final Debouncer leftConnectedDebounce = new Debouncer(0.5);
  private final Debouncer rightConnectedDebounce = new Debouncer(0.5);

  private double setpoint = 0.0;

  public ElevatorIOTalonFX() {
    left = new TalonFX(11);
    right = new TalonFX(10);
    config = new TalonFXConfiguration();

    right.setControl(new Follower(left.getDeviceID(), true));

    config.Audio.AllowMusicDurDisable = false;
    config.Audio.BeepOnBoot = true;
    config.Audio.BeepOnConfig = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = rampRate;

    config.MotionMagic.MotionMagicAcceleration = maxAcceleration;
    config.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
    config.MotionMagic.MotionMagicExpo_kA = kA.getAsDouble();
    config.MotionMagic.MotionMagicExpo_kV = kV.getAsDouble();

    config.Feedback.SensorToMechanismRatio = gearing / (2 * Math.PI * drumRadius);

    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.statorCurrent;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.supplyCurrent;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = supplyCurrentLow;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.0;

    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kP = kP.getAsDouble();
    config.Slot0.kI = kI.getAsDouble();
    config.Slot0.kD = kD.getAsDouble();

    config.Slot0.kV = kV.getAsDouble();
    config.Slot0.kS = kS.getAsDouble();
    config.Slot0.kA = kA.getAsDouble();
    config.Slot0.kG = kG.getAsDouble();

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> left.getConfigurator().apply(config, 0.25));

    position = left.getPosition();
    velocity = left.getVelocity();
    voltage = left.getMotorVoltage();
    statorCurrent = left.getTorqueCurrent();
    supplyCurrent = left.getSupplyCurrent();
    temperature = left.getDeviceTemp();
    followerVoltage = right.getMotorVoltage();
    followerStatorCurrent = right.getTorqueCurrent();
    followerSupplyCurrent = right.getSupplyCurrent();
    followerTemperature = right.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        position,
        velocity,
        voltage,
        supplyCurrent,
        statorCurrent,
        temperature,
        followerVoltage,
        followerSupplyCurrent,
        followerStatorCurrent,
        followerTemperature);

    left.optimizeBusUtilization();
    right.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        position,
        velocity,
        voltage,
        statorCurrent,
        supplyCurrent,
        temperature,
        followerVoltage,
        followerSupplyCurrent,
        followerStatorCurrent,
        followerTemperature);

    inputs.position = position.getValueAsDouble();
    inputs.velocity = velocity.getValueAsDouble();

    inputs.leftStatorCurrent = statorCurrent.getValueAsDouble();
    inputs.rightStatorCurrent = followerStatorCurrent.getValueAsDouble();

    inputs.leftSupplyCurrent = supplyCurrent.getValueAsDouble();
    inputs.rightSupplyCurrent = followerSupplyCurrent.getValueAsDouble();

    inputs.leftVolts = voltage.getValueAsDouble();
    inputs.rightVolts = followerVoltage.getValueAsDouble();

    inputs.leftTemp = temperature.getValueAsDouble();
    inputs.rightTemp = followerTemperature.getValueAsDouble();

    inputs.targetHeight = setpoint;

    inputs.leftConnected =
        leftConnectedDebounce.calculate(
            BaseStatusSignal.isAllGood(voltage, statorCurrent, supplyCurrent, temperature));
    inputs.rightConnected =
        rightConnectedDebounce.calculate(
            BaseStatusSignal.isAllGood(
                followerVoltage,
                followerStatorCurrent,
                followerSupplyCurrent,
                followerTemperature));

    // Setting up PID & FF Values
    if (kP.hasChanged(hashCode())) {
      resetPIDValues();
    }

    if (kI.hasChanged(hashCode())) {
      resetPIDValues();
    }

    if (kD.hasChanged(hashCode())) {
      resetPIDValues();
    }

    if (kV.hasChanged(hashCode())) {
      resetFFValues();
    }

    if (kS.hasChanged(hashCode())) {
      resetFFValues();
    }

    if (kA.hasChanged(hashCode())) {
      resetFFValues();
    }

    if (kG.hasChanged(hashCode())) {
      resetFFValues();
    }
  }

  private void resetPIDValues() {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.withKP(kP.getAsDouble());
    slot0Configs.withKI(kI.getAsDouble());
    slot0Configs.withKD(kD.getAsDouble());
    left.getConfigurator().apply(slot0Configs, 0.25);
  }

  private void resetFFValues() {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.withKV(kV.getAsDouble());
    slot0Configs.withKA(kA.getAsDouble());
    slot0Configs.withKG(kG.getAsDouble());
    slot0Configs.withKS(kS.getAsDouble());
    left.getConfigurator().apply(slot0Configs, 0.25);
  }

  @Override
  public void setControl(double position) {
    setpoint = position;
    left.setControl(positionTorque.withPosition(position));
  }

  @Override
  public void setVolts(double volts) {
    left.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void resetEncoder() {
    left.setPosition(0.0);
  }

  @Override
  public double positionError() {
    return left.getClosedLoopError().getValueAsDouble();
  }
}
