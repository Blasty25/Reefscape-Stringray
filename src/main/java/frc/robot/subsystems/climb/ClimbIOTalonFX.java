package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.RobotMap.ClimbMap;

public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX talon = new TalonFX(ClimbMap.talon, "rio");

  private final StatusSignal<AngularVelocity> motorVelocityRadPerSec = talon.getVelocity();
  private final StatusSignal<Temperature> motorTempCelsius = talon.getDeviceTemp();
  private final StatusSignal<Current> motorSupplyCurrentAmps = talon.getSupplyCurrent();
  private final StatusSignal<Current> motorStatorCurrentAmps = talon.getStatorCurrent();
  private final StatusSignal<Voltage> motorAppliedVolts = talon.getMotorVoltage();
  private final StatusSignal<Angle> motorPositionRad = talon.getPosition();

  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final MotionMagicVoltage motionMagic =
      new MotionMagicVoltage(ClimbConstants.stow.getRadians());

  private final Debouncer connectedDebounce = new Debouncer(0.5);

  private double targetPositionRad;

  public ClimbIOTalonFX() {
    final TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = ClimbConstants.kPTalon;

    config.MotionMagic.MotionMagicCruiseVelocity = (6000 / 60) / ClimbConstants.gearing;
    config.MotionMagic.MotionMagicAcceleration = (6000 / 60) / (ClimbConstants.gearing * 0.01);

    config.Feedback.SensorToMechanismRatio = ClimbConstants.gearing;

    config.CurrentLimits.StatorCurrentLimit = ClimbConstants.talonStatorCurrent;
    config.CurrentLimits.StatorCurrentLimitEnable = false;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    talon.getConfigurator().apply(config);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        motorVelocityRadPerSec,
        motorTempCelsius,
        motorAppliedVolts,
        motorSupplyCurrentAmps,
        motorStatorCurrentAmps,
        motorPositionRad);
    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimbIOInputsAutoLogged inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            motorVelocityRadPerSec,
            motorTempCelsius,
            motorSupplyCurrentAmps,
            motorStatorCurrentAmps,
            motorPositionRad,
            motorAppliedVolts);

    inputs.motorConnected = connectedDebounce.calculate(status.isOK());
    inputs.motorPositionRad = motorPositionRad.getValueAsDouble();
    inputs.targetPositionRad = this.targetPositionRad;
    inputs.motorTempCelsius = motorTempCelsius.getValue().in(Units.Celsius);
    inputs.motorStatorCurrentAmps = motorStatorCurrentAmps.getValueAsDouble();
    inputs.motorSupplyCurrentAmps = motorSupplyCurrentAmps.getValueAsDouble();
    inputs.motorVelocityRadPerSec = motorVelocityRadPerSec.getValueAsDouble();
    inputs.motorAppliedVolts = motorAppliedVolts.getValueAsDouble();

    Constants.logMotorStatus("Climb", inputs.motorConnected);
    if (!Constants.getMotorStatus("Climb")) setVoltage(0.0);
  }

  @Override
  public void setPosition(Rotation2d position) {
    this.targetPositionRad = position.getRadians();
    talon.setControl(motionMagic.withPosition(position.getRadians()));
  }

  @Override
  public void setVoltage(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    setVoltage(0);
  }

  @Override
  public void resetEncoder() {
    talon.setPosition(0.0);
  }

  @Override
  public void resetEncoder(double position) {
    talon.setPosition(position);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    talon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
