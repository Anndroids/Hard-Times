package frc.robot;

import static frc.robot.Constants.ElevatorConstants.CORAL_POSITION_L1_POSITION;
import static frc.robot.Constants.ElevatorConstants.CORAL_POSITION_L2_POSITION;
import static frc.robot.Constants.ElevatorConstants.CORAL_POSITION_L3_POSITION;
import static frc.robot.Constants.ElevatorConstants.CORAL_POSITION_STOW_DOWN_POSITION;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_L1_POSITION;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_L2_POSITION;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_L3_POSITION;

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
        NamedCommands.registerCommand("CoralOuttake", coralOuttake());
        NamedCommands.registerCommand("CoralStow", coralStow());
        NamedCommands.registerCommand("CoralIntake", coralIntake());
        NamedCommands.registerCommand("ElevatorL3", elevatorToL3());
        NamedCommands.registerCommand("ElevatorL2", elevatorToL2());
    }

    public Command coralOuttake() {
        return coralIntake.run(() -> coralIntake.run(coralIntake::outtake).finallyDo(coralIntake::stop));
    }

    public Command coralIntake() {
        return coralIntake.run(() -> coralIntake.run(coralIntake::intake).finallyDo(coralIntake::stop));
    }

    public Command coralStow() {
        return coralIntake.run(() -> elevatorSubsystem.goToPosition(CORAL_POSITION_STOW_DOWN_POSITION, ELEVATOR_L1_POSITION));
    }

    public Command elevatorToL1() {
        return elevatorSubsystem.run(() -> elevatorSubsystem.goToPosition(CORAL_POSITION_L1_POSITION, ELEVATOR_L1_POSITION));
    }

    public Command elevatorToL2() {
        return elevatorSubsystem.run(() -> elevatorSubsystem.goToPosition(CORAL_POSITION_L2_POSITION, ELEVATOR_L2_POSITION));
    }

    public Command elevatorToL3() {
        return elevatorSubsystem.run(() -> elevatorSubsystem.goToPosition(CORAL_POSITION_L3_POSITION, ELEVATOR_L3_POSITION));
    }

}
