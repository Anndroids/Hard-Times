package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CoralIntakeConstants.DEVICE_ID_CANRANGE;
import static frc.robot.Constants.CoralIntakeConstants.DEVICE_ID_CORAL_INTAKE;
import static frc.robot.Constants.CoralIntakeConstants.HOLD_VOLTAGE;
import static frc.robot.Constants.CoralIntakeConstants.INTAKE_VOLTAGE;
import static frc.robot.Constants.CoralIntakeConstants.OUTTAKE_VOLTAGE;
import static frc.robot.Constants.CoralIntakeConstants.SCORE_VOLTAGE;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class CoralIntake extends SubsystemBase {

    private TalonFX coralIntakeMotor = new TalonFX(DEVICE_ID_CORAL_INTAKE);
    private CANrange canRange = new CANrange(DEVICE_ID_CANRANGE);
    private StatusSignal<Boolean> hasCoral = canRange.getIsDetected();

    private VoltageOut coralIntakeControl = new VoltageOut(0.0);
    private VoltageOut sysIdControl = new VoltageOut(0.0);    

    private SysIdRoutine coralIntakeSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, state -> SignalLogger.writeString("State", state.toString())),
            new Mechanism((voltage) -> coralIntakeMotor.setControl(sysIdControl.withOutput(voltage.in(Volts))), null, this));

    public CoralIntake() {
        var coralIntakeConfig = new TalonFXConfiguration();
        coralIntakeConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive);
        coralIntakeConfig.HardwareLimitSwitch
                .withForwardLimitEnable(true)
                .withForwardLimitRemoteCANrange(canRange)
                .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen);

        coralIntakeMotor.getConfigurator().apply(coralIntakeConfig);

        var canRangeConfig = new CANrangeConfiguration();
        canRangeConfig.ProximityParams.withProximityThreshold(Meters.of(0.22));

        canRange.getConfigurator().apply(canRangeConfig);
    }

    public Command sysIdCoralIntakeDynamicCommand(Direction direction) {
        return coralIntakeSysIdRoutine.dynamic(direction).withName("sysId Coral Intake Dynamic "+ direction).finallyDo(this::stop);
    }

    public Command sysIdCoralIntakeQuasistaticCommand(Direction direction) {
        return coralIntakeSysIdRoutine.quasistatic(direction).withName("sysId Coral Intake Quasistatic "+ direction).finallyDo(this::stop);
    }

    public void intake() {
        runCoralIntake(INTAKE_VOLTAGE);
    }

    public void hold() {
        runCoralIntake(HOLD_VOLTAGE);
    }

    public void stop() {
        coralIntakeMotor.stopMotor();
    }

    public void score() {
        runCoralIntake(SCORE_VOLTAGE);
    }

    public void outtake() {
        runCoralIntake(OUTTAKE_VOLTAGE);
    }

    public boolean hasCoral() {
        hasCoral.refresh();
        return hasCoral.getValue();
    }

    private void runCoralIntake(Voltage voltage) {
        coralIntakeMotor.setControl(coralIntakeControl.withOutput(voltage));
    }

}
