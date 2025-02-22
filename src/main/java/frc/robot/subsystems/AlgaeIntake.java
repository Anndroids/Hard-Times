// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Volts;
// import static frc.robot.Constants.AlgaeIntakeConstants.ALGAE_INTAKE_MOTOR_ID;
// import static frc.robot.Constants.AlgaeIntakeConstants.INWARD_VELOCITY;
// import static frc.robot.Constants.AlgaeIntakeConstants.POINT_VELOCITY;
// import static frc.robot.Constants.AlgaeIntakeConstants.SPIT_VELOCITY;
// import static frc.robot.Constants.AlgaeIntakeConstants.STOP_VELOCITY;

// import com.ctre.phoenix6.SignalLogger;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

// public class AlgaeIntake extends SubsystemBase {
// private TalonFX algaeIntakeMotor = new TalonFX(ALGAE_INTAKE_MOTOR_ID);

//     private VelocityVoltage algaeIntakeControl = new VelocityVoltage(0.0);
//     private VoltageOut sysIdControl = new VoltageOut(0.0);
    

//     private SysIdRoutine algaeIntakeSysIdRoutine = new SysIdRoutine(
//             new SysIdRoutine.Config(null, null, null, state -> SignalLogger.writeString("State", state.toString())),
//             new Mechanism((voltage) -> algaeIntakeMotor.setControl(sysIdControl.withOutput(voltage.in(Volts))), null, this));

//     public AlgaeIntake() {
//         var algaeIntakeConfig = new TalonFXConfiguration();
//         algaeIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

//         algaeIntakeMotor.getConfigurator().apply(algaeIntakeConfig);
//     }

//     public Command sysIdAlgaeIntakeDynamicCommand(Direction direction) {
//         return algaeIntakeSysIdRoutine.dynamic(direction).withName("sysId Algae Intake Dynamic "+ direction).finallyDo(this::stop);
//     }

//     public Command sysIdAlgaeIntakeQuasistaticCommand(Direction direction) {
//         return algaeIntakeSysIdRoutine.quasistatic(direction).withName("sysId Algae Intake Quasistatic "+ direction).finallyDo(this::stop);
//     }

//     public void intake() {
//         runAlgaeIntake(INWARD_VELOCITY);
//     }

//     public void hold() {
//         runAlgaeIntake(STOP_VELOCITY);
//     }

//     public void stop() {
//         algaeIntakeMotor.stopMotor();
//     }

//     public void score() {
//         runAlgaeIntake(POINT_VELOCITY);
//     }

//     public void outtake() {
//         runAlgaeIntake(SPIT_VELOCITY);
//     }

//     private void runAlgaeIntake(AngularVelocity velocity) {
//         algaeIntakeMotor.setControl(algaeIntakeControl.withVelocity(velocity));
//     }

// }

