package frc.robot.subsystems;

import static frc.robot.Constants.CANdiConstants.DEVICE_ID_INSIDE_CANDI;
import static frc.robot.Constants.CANdiConstants.DEVICE_ID_OUTSIDE_CANDI;
import static frc.robot.Constants.CANdiConstants.OUTSIDE_CANDI_PWM1_OFFSET;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.PWM1Configs;
import com.ctre.phoenix6.hardware.CANdi;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdisSubsystem extends SubsystemBase {

    private final CANdi insideCANdi;
    private final CANdi outsideCANdi;

    public CANdisSubsystem() {
        CANdiConfiguration config = new CANdiConfiguration()
                .withPWM1(new PWM1Configs()
                        .withSensorDirection(false)
                        .withAbsoluteSensorDiscontinuityPoint(1)
                        .withAbsoluteSensorOffset(OUTSIDE_CANDI_PWM1_OFFSET));
        insideCANdi = new CANdi(DEVICE_ID_INSIDE_CANDI);
        outsideCANdi = new CANdi(DEVICE_ID_OUTSIDE_CANDI);
        outsideCANdi.getConfigurator().apply(config);
    }

    public StatusSignal<Angle> getClimbPosition() {
        return outsideCANdi.getPWM1Position();
    }

    public StatusSignal<Boolean> getElevatorLowerLimitSwitch() {
        return outsideCANdi.getS2Closed();
    }

    public StatusSignal<Boolean> getElevatorUpperLimitSwitch() {
        return insideCANdi.getS1Closed();
    }

    public StatusSignal<Angle> getCoralPosition() {
        return insideCANdi.getPWM2Position();
    }

    public CANdi getInsideCANdi() {
        return insideCANdi;
    }

}
