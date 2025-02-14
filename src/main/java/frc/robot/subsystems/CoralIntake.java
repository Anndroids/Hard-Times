package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CoralIntakeConstants.CORAL_INTAKE_MOTOR_ID;
import static frc.robot.Constants.CoralIntakeConstants.HOLD_VELOCITY;
import static frc.robot.Constants.CoralIntakeConstants.INTAKE_VELOCITY;
import static frc.robot.Constants.CoralIntakeConstants.OUTTAKE_VELOCITY;
import static frc.robot.Constants.CoralIntakeConstants.SCORE_VELOCITY;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class CoralIntake extends SubsystemBase {

    private TalonFX coralIntakeMotor = new TalonFX(CORAL_INTAKE_MOTOR_ID);

    private VelocityVoltage coralIntakeControl = new VelocityVoltage(0.0);
    private VoltageOut sysIdControl = new VoltageOut(0.0);
    

    private SysIdRoutine coralIntakeSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, state -> SignalLogger.writeString("State", state.toString())),
            new Mechanism((voltage) -> coralIntakeMotor.setControl(sysIdControl.withOutput(voltage.in(Volts))), null, this));

    public CoralIntake() {
        var coralIntakeConfig = new TalonFXConfiguration();
        coralIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        coralIntakeMotor.getConfigurator().apply(coralIntakeConfig);
    }

    public Command sysIdCoralIntakeDynamicCommand(Direction direction) {
        return coralIntakeSysIdRoutine.dynamic(direction).withName("sysId Coral Intake Dynamic "+ direction).finallyDo(this::stop);
    }

    public Command sysIdCoralIntakeQuasistaticCommand(Direction direction) {
        return coralIntakeSysIdRoutine.quasistatic(direction).withName("sysId Coral Intake Quasistatic "+ direction).finallyDo(this::stop);
    }

    public void intake() {
        runCoralIntake(INTAKE_VELOCITY);
    }

    public void hold() {
        runCoralIntake(HOLD_VELOCITY);
    }

    public void stop() {
        coralIntakeMotor.stopMotor();
    }

    public void score() {
        runCoralIntake(SCORE_VELOCITY);
    }

    public void outtake() {
        runCoralIntake(OUTTAKE_VELOCITY);
    }

    private void runCoralIntake(AngularVelocity velocity) {
        coralIntakeMotor.setControl(coralIntakeControl.withVelocity(velocity));
    }

}
