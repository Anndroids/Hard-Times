package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ClimbConstants.CLIMB_DEPLOY_VOLTAGE;
import static frc.robot.Constants.ClimbConstants.CLIMB_GEAR_RATIO;
import static frc.robot.Constants.ClimbConstants.CLIMB_RETRACT_VOLTAGE;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB_FOLLOWER;
import static frc.robot.Constants.ClimbConstants.DEVICE_ID_CLIMB_LEADER;
import static frc.robot.Constants.ClimbConstants.FORWARD_LIMIT_CLIMB;
import static frc.robot.Constants.ClimbConstants.REVERSE_LIMIT_CLIMB;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANdiConstants;

public class ClimbSubsystem extends SubsystemBase {

    private final TalonFX climbLeader;
    private final TalonFX climbFollower;

    // private final StatusSignal<Angle> climbAngle;

    private final VoltageOut climbVoltage = new VoltageOut(0.0);
    private final StatusSignal<Angle> climbAngle;

    public ClimbSubsystem(StatusSignal<Angle> climbAngle) {
        this.climbAngle = climbAngle;

        climbLeader = new TalonFX(DEVICE_ID_CLIMB_LEADER);
        climbFollower = new TalonFX(DEVICE_ID_CLIMB_FOLLOWER);

        var climbConfiguration = new TalonFXConfiguration();
        climbConfiguration.MotorOutput
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        climbConfiguration.ClosedLoopGeneral
                .withContinuousWrap(true);
        climbConfiguration.Feedback
                .withFeedbackRemoteSensorID(CANdiConstants.DEVICE_ID_OUTSIDE_CANDI)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANdiPWM1);
        // climbConfiguration.SoftwareLimitSwitch
        //         .withForwardSoftLimitEnable(true)
        //         .withReverseSoftLimitEnable(true)
        //         .withForwardSoftLimitThreshold(FORWARD_LIMIT_CLIMB)
        //         .withReverseSoftLimitThreshold(REVERSE_LIMIT_CLIMB);

        climbLeader.getConfigurator().apply(climbConfiguration);
        climbConfiguration.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        climbFollower.getConfigurator().apply(climbConfiguration);
        climbFollower.setControl(new Follower(DEVICE_ID_CLIMB_LEADER, true));
    }

    public void deploy() {
        if (climbAngle.refresh().getValue().in(Rotations) % 1 >= FORWARD_LIMIT_CLIMB) {
            climbVoltage.withOutput(0);
        } else {
            climbVoltage.withOutput(CLIMB_DEPLOY_VOLTAGE);
        }
    }

    public void retract() {
        if (climbAngle.refresh().getValue().in(Rotations) % 1 <= REVERSE_LIMIT_CLIMB) {
            climbVoltage.withOutput(0);
        } else {
            climbVoltage.withOutput(CLIMB_RETRACT_VOLTAGE);
        }
    }

    public void stop() {
        climbVoltage.withOutput(Volts.of(0));
    }

    @Override
    public void periodic() {
        climbLeader.setControl(climbVoltage);
    }

}
