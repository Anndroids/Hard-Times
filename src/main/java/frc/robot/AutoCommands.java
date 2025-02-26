package frc.robot;

import static frc.robot.Constants.ElevatorConstants.CORAL_POSITION_L1_POSITION;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_L1_POSITION;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdisSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoCommands {

    private final CommandSwerveDrivetrain drivetrainSubsystem;
    private final CANdisSubsystem candisSubsystem;
    private final CoralIntake coralIntake;
    private final ElevatorSubsystem elevatorSubsystem;
    
    public AutoCommands(
        CommandSwerveDrivetrain drivetrainSubsystem,
        CANdisSubsystem candisSubsystem,
        CoralIntake coralIntake,
        ElevatorSubsystem elevatorSubsystem
    ) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.candisSubsystem = candisSubsystem;
        this.coralIntake = coralIntake;
        this.elevatorSubsystem = elevatorSubsystem;
    }

    public void registerPPNamedCommands() {
        NamedCommands.registerCommand("ElevatorL1", elevatorToL1());
    }

    public Command elevatorToL1() {
        return elevatorSubsystem.run(() -> elevatorSubsystem.goToPosition(CORAL_POSITION_L1_POSITION, ELEVATOR_L1_POSITION));
    }

}
