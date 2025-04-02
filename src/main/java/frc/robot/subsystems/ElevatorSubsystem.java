package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.ElevatorConstants.CORAL_POSITION_KG;
import static frc.robot.Constants.ElevatorConstants.CORAL_POSITION_KP;
import static frc.robot.Constants.ElevatorConstants.DEVICE_ID_CORAL_POSITION;
import static frc.robot.Constants.ElevatorConstants.DEVICE_ID_ELEVATOR_FOLLOWER;
import static frc.robot.Constants.ElevatorConstants.DEVICE_ID_ELEVATOR_LEADER;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_KG;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_KP;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private final TalonFX coralPosition = new TalonFX(DEVICE_ID_CORAL_POSITION);
    private final TalonFX elevatorLeader = new TalonFX(DEVICE_ID_ELEVATOR_LEADER);
    private final TalonFX elevatorFollower = new TalonFX(DEVICE_ID_ELEVATOR_FOLLOWER);

    private final MotionMagicVoltage elevatorControl = new MotionMagicVoltage(0.0);
    private final MotionMagicVoltage coralPositionControl = new MotionMagicVoltage(Rotations.of(0));

    public ElevatorSubsystem(CANdi encoderCANdi) {
        var coralPositionConfiguration = new TalonFXConfiguration();
        coralPositionConfiguration.MotorOutput
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        coralPositionConfiguration.Feedback
                .withRemoteCANdiPwm2(encoderCANdi);
        coralPositionConfiguration.Slot0
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKP(CORAL_POSITION_KP)
                .withKG(CORAL_POSITION_KG);
        coralPositionConfiguration.ClosedLoopGeneral
                .withContinuousWrap(false);
        coralPositionConfiguration.MotionMagic
                .withMotionMagicAcceleration(10)
                .withMotionMagicCruiseVelocity(1.5);
        coralPositionConfiguration.SoftwareLimitSwitch
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(0.251)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(-0.251);
        coralPosition.getConfigurator().apply(coralPositionConfiguration);

        var elevatorConfiguration = new TalonFXConfiguration();
        elevatorConfiguration.MotorOutput
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        elevatorConfiguration.Slot0
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKP(ELEVATOR_KP)
                .withKG(ELEVATOR_KG);
        elevatorConfiguration.MotionMagic
                .withMotionMagicAcceleration(500)
                .withMotionMagicCruiseVelocity(100);
        
        elevatorLeader.getConfigurator().apply(elevatorConfiguration);
        elevatorFollower.getConfigurator().apply(elevatorConfiguration);
        elevatorFollower.setControl(new Follower(DEVICE_ID_ELEVATOR_LEADER, false));
    }

    public void goToPosition(Angle coralPositionAngle, double elevatorPosition) {
        coralPositionControl.withPosition(coralPositionAngle);
        elevatorControl.withPosition(elevatorPosition);
        coralPosition.setControl(coralPositionControl);
        elevatorLeader.setControl(elevatorControl);
    }

}