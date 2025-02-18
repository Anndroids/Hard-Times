package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.AngularVelocity;

public class Constants {
    
    public static class CoralIntakeConstants {
    
        public static final int CORAL_INTAKE_MOTOR_ID = 11;

        public static final AngularVelocity INTAKE_VELOCITY = RadiansPerSecond.of(5);
        public static final AngularVelocity SCORE_VELOCITY = RadiansPerSecond.of(-5);
        public static final AngularVelocity HOLD_VELOCITY = RadiansPerSecond.of(5);
        public static final AngularVelocity OUTTAKE_VELOCITY = RadiansPerSecond.of(-5);
        
    }

    public static class ElevatorConstants {
    
        public static final int DEVICE_ID_ELEVATOR_MOTOR_LEADER = 16;
        public static final int DEVICE_ID_ELEVATOR_MOTOR_FOLLOWER = 13;
        public static final int DEVICE_ID_ELEVATOR_CANDI = 17;
        public static final int DEVICE_ID_ARM_CANDI = 0;
        public static final int DEVICE_ID_ARM_MOTOR = 0;
        public static final DistanceUnit ELEVATOR_BASE_HEIGHT = (Meters);
        public static final DistanceUnit ELEVATOR_DEFAULT_HEIGHT = (Meters);
        public static final int LEVEL_1_HEIGHT = 2;
        public static final int LEVEL_2_HEIGHT = 3;
        public static final int LEVEL_3_HEIGHT = 4;
        public static final int LEVEL_4_HEIGHT = 5;
        public static final int LEVEL_2_ANGLE = 1;
        public static final int LEVEL_3_ANGLE = 2;
        public static final int LEVEL_4_ANGLE = 3;
        public static final int INTAKE_ANGLE = 2;
        public static final DistanceUnit ARM_PIVOT_LENGTH = (Meters);
        public static final int ELEVATOR_SLOT_CONFIGS = 3;
        public static final int ELEVATOR_SUPPLY_CURRENT_LIMIT = 3;
        public static final int ELEVATOR_STATOR_CURRENT_LIMIT = 4;
        public static final int ELEVATOR_MOTION_MAGIC_CONFIGS = 9;
        public static final DistanceUnit ELEVATOR_TOP_LIMIT = (Meters);
        public static final DistanceUnit ELEVATOR_BOTTOM_LIMIT = (Meters);
        public static final DistanceUnit ELEVATOR_DISTANCE_PER_ROTATION = (Meters);
        public static final int ARM_SLOT_CONFIGS = 9;
        public static final int ARM_SUPPLY_CURRENT_LIMIT = 9;
        public static final int ARM_STATOR_CURRENT_LIMIT = 9;
        public static final int ARM_MOTION_MAGIC_CONFIGS = 9;
        public static final int ARM_ROTOR_TO_SENSOR_RATIO = 9;
        public static final int ELEVATOR_POSITION_TOLERANCE = 9;
        public static final int ARM_POSITION_TOLERANCE = 9;
    }
    public static class AlgaeIntakeConstants {
            public static final int ALGAE_INTAKE_MOTOR_ID = 6;
    
            public static final AngularVelocity INWARD_VELOCITY = RadiansPerSecond.of(5);
            public static final AngularVelocity POINT_VELOCITY = RadiansPerSecond.of(-5);
            public static final AngularVelocity STOP_VELOCITY = RadiansPerSecond.of(5);
            public static final AngularVelocity SPIT_VELOCITY = RadiansPerSecond.of(-5);
    }
}
