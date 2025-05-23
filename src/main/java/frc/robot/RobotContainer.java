// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.TriggerEvent;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANdisSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final MedianFilter joystickLeftYFilter = new MedianFilter(10);
    private final MedianFilter joystickLeftXFilter = new MedianFilter(10);
    private final MedianFilter joystickRightXFilter = new MedianFilter(10);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CoralIntake coralIntake = new CoralIntake();
    public final CANdisSubsystem candisSubsystem = new CANdisSubsystem();
    public final ClimbSubsystem climbSubsystem = new ClimbSubsystem(candisSubsystem.getClimbPosition());
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(candisSubsystem.getInsideCANdi());

    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    private final AutoCommands autoCommands = new AutoCommands(drivetrain, candisSubsystem, coralIntake, elevatorSubsystem);

    public RobotContainer() {
        autoCommands.registerPPNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystickLeftYFilter.calculate(-joystick.getLeftY() * MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(joystickLeftXFilter.calculate(-joystick.getLeftX() * MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate(joystickRightXFilter.calculate(-joystick.getRightX() * MaxAngularRate)) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

       // joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            //forwardStraight.withVelocityX(0.5).withVelocityY(0))
        //);
        //joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            //forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        //);

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        operator.rightTrigger().whileTrue(climbSubsystem.run(climbSubsystem::deploy).finallyDo(climbSubsystem::stop));
        operator.rightTrigger().onTrue(elevatorSubsystem.run(() -> elevatorSubsystem.goToPosition(Rotations.of(-0.07), 0)));
        operator.leftTrigger().whileTrue(climbSubsystem.run(climbSubsystem::retract).finallyDo(climbSubsystem::stop));
        operator.leftTrigger().onTrue(elevatorSubsystem.run(() -> elevatorSubsystem.goToPosition(Rotations.of(-0.07), 0)));
        //operator.rightTrigger().whileTrue(coralIntake.run(coralIntake::intake).finallyDo(coralIntake::stop));
        operator.rightBumper().whileTrue(coralIntake.run(coralIntake::intake).finallyDo(coralIntake::stop));
        operator.rightBumper().whileTrue(elevatorSubsystem.run(() -> elevatorSubsystem.goToPosition(Rotations.of(0.09), 0)));
        //operator.pov(0).whileTrue(elevatorSubsystem.run(() -> elevatorSubsystem.goToPosition(Rotations.of(0.09), 0)));
        operator.leftBumper().whileTrue(coralIntake.run(coralIntake::outtake).finallyDo(coralIntake::stop));
        operator.a().onTrue(elevatorSubsystem.run(() -> elevatorSubsystem.goToPosition(Rotations.of(-0.07), 0)));
        operator.x().onTrue(elevatorSubsystem.run(() -> elevatorSubsystem.goToPosition(Rotations.of(-0.07), 20.1)));
        operator.y().onTrue(elevatorSubsystem.run(() -> elevatorSubsystem.goToPosition(Rotations.of(-0.05), 51.2)));
        operator.b().and(() -> !coralIntake.hasCoral()).onTrue(elevatorSubsystem.run(() -> elevatorSubsystem.goToPosition(Rotations.of(-0.23), 0)));
        elevatorSubsystem.setDefaultCommand(elevatorSubsystem.run(() -> elevatorSubsystem.goToPosition(Rotations.of(0.187), 0)));
        coralIntake.setDefaultCommand(coralIntake.run(coralIntake::hold).finallyDo(coralIntake::stop));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
