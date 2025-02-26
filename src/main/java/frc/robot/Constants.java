package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class Constants {

    public static class CoralIntakeConstants {

        public static final int DEVICE_ID_CORAL_INTAKE = 6;
        public static final int DEVICE_ID_CANRANGE = 18;

        public static final Voltage INTAKE_VOLTAGE = Volts.of(4.0);
        public static final Voltage SCORE_VOLTAGE = Volts.of(-4.0);
        public static final Voltage HOLD_VOLTAGE = Volts.of(1.0);
        public static final Voltage OUTTAKE_VOLTAGE = Volts.of(-4.0);

    }

    public static class ElevatorConstants {
        public static final int DEVICE_ID_CORAL_POSITION = 13;

        public static final double CORAL_POSITION_KP = 14.0;
        public static final double CORAL_POSITION_KG = 0.45;

        public static final int DEVICE_ID_ELEVATOR_LEADER = 7;
        public static final int DEVICE_ID_ELEVATOR_FOLLOWER = 4;

        public static final double ELEVATOR_KP = 1.0;
        public static final double ELEVATOR_KG = 0.15;

        public static final double ELEVATOR_L1_POSITION = 0;
        public static final Angle CORAL_POSITION_L1_POSITION = Rotations.of(-0.07);
        public static final double ELEVATOR_L2_POSITION = 20.1;
        public static final Angle CORAL_POSITION_L2_POSITION = Rotations.of(-0.07);
        public static final double ELEVATOR_L3_POSITION = 51.2;
        public static final Angle CORAL_POSITION_L3_POSITION = Rotations.of(-0.07);
        public static final double ELEVATOR_INTAKE_POSITION = 0;
        public static final Angle CORAL_POSITION_INTAKE_POSITION = Rotations.of(0.09);
        public static final double ELEVATOR_STOW_UP_POSITION = 0;
        public static final Angle CORAL_POSITION_STOW_UP_POSITION = Rotations.of(0.25);
        public static final double ELEVATOR_STOW_DOWN_POSITION = 0;
        public static final Angle CORAL_POSITION_STOW_DOWN_POSITION = Rotations.of(-0.23);
    }

    public static class CANdiConstants {

        public static final int DEVICE_ID_INSIDE_CANDI = 19;
        public static final int DEVICE_ID_OUTSIDE_CANDI = 20;

    }

    public static class ClimbConstants {

        public static final int DEVICE_ID_CLIMB_LEADER = 2;
        public static final int DEVICE_ID_CLIMB_FOLLOWER = 5;

        public static final double CLIMB_GEAR_RATIO = 120.0;

        public static final double FORWARD_LIMIT_CLIMB = 0.511 * CLIMB_GEAR_RATIO;
        public static final double REVERSE_LIMIT_CLIMB = 0.005 * CLIMB_GEAR_RATIO;

        public static final Voltage CLIMB_DEPLOY_VOLTAGE = Volts.of(4.0);
        public static final Voltage CLIMB_RETRACT_VOLTAGE = Volts.of(-2.0);

    }

}
