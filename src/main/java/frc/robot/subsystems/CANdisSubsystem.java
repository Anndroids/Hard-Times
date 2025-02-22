package frc.robot.subsystems;

import static frc.robot.Constants.CANdiConstants.DEVICE_ID_INSIDE_CANDI;
import static frc.robot.Constants.CANdiConstants.DEVICE_ID_OUTSIDE_CANDI;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANdi;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdisSubsystem extends SubsystemBase {

    private final CANdi insideCANdi;
    private final CANdi outsideCANdi;
    
    public CANdisSubsystem() {
        insideCANdi = new CANdi(DEVICE_ID_INSIDE_CANDI);
        outsideCANdi = new CANdi(DEVICE_ID_OUTSIDE_CANDI);
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

}
