    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

    package frc.robot;

    import static edu.wpi.first.units.Units.*;

    import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
    import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
    import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
    import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlgaePivotCommands;
import frc.robot.commands.CommandSwerveDrivetrain;
    import frc.robot.commands.CommandSwerveDrivetrain.branchSide;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.EndEffectorCommands;
import frc.robot.commands.IntakeRollersCommands;
import frc.robot.commands.SensorRangeCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeDetectRangeSubsystem;
import frc.robot.subsystems.AlgaePivotSubsystem;
import frc.robot.subsystems.EECoralRangeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.IntakeRangeSubsystem;
import frc.robot.subsystems.IntakeRollersSubsystems;
import com.pathplanner.lib.auto.AutoBuilder;

    public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


        private final ElevatorSubsystem elevator = new ElevatorSubsystem();
        private final AlgaePivotSubsystem algaepivot = new AlgaePivotSubsystem();
        private final EndEffectorSubsystem endEffector = new EndEffectorSubsystem();    
        private final IntakeRollersSubsystems intakeRollers = new IntakeRollersSubsystems();
        private final EECoralRangeSubsystem eeCoral = new EECoralRangeSubsystem();
        private final AlgaeDetectRangeSubsystem algaeDetect = new AlgaeDetectRangeSubsystem();
        private final IntakeRangeSubsystem intakeRange = new IntakeRangeSubsystem();
        private final SendableChooser<Command> autoChooser;



        private RobotMode selectedMode = RobotMode.NONE;
        private int modeStep = 0;

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController joystick = new CommandXboxController(0);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        public RobotContainer() {
            configureBindings();
            NamedCommands.registerCommand("ElevatorL3", ElevatorCommands.goToLevel3(elevator));
            NamedCommands.registerCommand("RunEE", EndEffectorCommands.run(endEffector, 0.6));
            NamedCommands.registerCommand("RunEEIntake", EndEffectorCommands.run(endEffector, 0.3));
            NamedCommands.registerCommand("ResetELEVATOR", ElevatorCommands.reset(elevator));
            NamedCommands.registerCommand("StopEE", EndEffectorCommands.stop(endEffector));
            NamedCommands.registerCommand("StowAlgae", AlgaePivotCommands.stow(algaepivot));
            NamedCommands.registerCommand("ScoreAlgae", AlgaePivotCommands.runEE(algaepivot));
            NamedCommands.registerCommand("IntakeRunEE", EndEffectorCommands.run(endEffector, 0.3));
            NamedCommands.registerCommand("StopIntakeRollers", IntakeRollersCommands.stop(intakeRollers));
            NamedCommands.registerCommand("SlowIntakeRollers", IntakeRollersCommands.run(intakeRollers, 0.15));
            NamedCommands.registerCommand("IntakeRollers", IntakeRollersCommands.run(intakeRollers, 0.55));
            NamedCommands.registerCommand("EESensor", SensorRangeCommands.waitForCoral(eeCoral, 0, 100));
            NamedCommands.registerCommand("IntakeSensor", SensorRangeCommands.waitForIntake(intakeRange, 0, 280));
            autoChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        private void configureBindings() {
            // Note that X is defined as forward according to WPILib convention,
            // and Y is defined as to the left according to WPILib convention.
            drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.55) // Drive forward with negative Y (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.55) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
            );

            // Idle while the robot is disabled. This ensures the configured
            // neutral mode is applied to the drive motors while disabled.
            final var idle = new SwerveRequest.Idle();
            RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true)
            );

            // Run SysId routines when holding back/start and X/Y.
            // Note that each routine should be run exactly once in a single log.
            joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
            joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
            joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
            joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

            drivetrain.registerTelemetry(logger::telemeterize);

            //AutoAlign
            joystick.rightBumper().whileTrue(drivetrain.defer(() -> drivetrain.autoAlign(drivetrain.getBranchPose(branchSide.rightBranch))));
            joystick.leftBumper().whileTrue(drivetrain.defer(() -> drivetrain.autoAlign(drivetrain.getBranchPose(branchSide.leftBranch))));

            // Intake
            joystick.leftTrigger().onTrue(
                Commands.sequence(
                    AlgaePivotCommands.stow(algaepivot),
                    AlgaePivotCommands.runEE(algaepivot),
                    ElevatorCommands.reset(elevator),
                    EndEffectorCommands.stop(endEffector),
                    IntakeRollersCommands.stop(intakeRollers),
                    IntakeRollersCommands.run(intakeRollers, 0.5),
                    EndEffectorCommands.run(endEffector, 0.3),
                    SensorRangeCommands.waitForIntake(intakeRange, 0, 280),
                    IntakeRollersCommands.run(intakeRollers, 0.15),
                    SensorRangeCommands.waitForCoral(eeCoral, 0, 100),
                    Commands.waitSeconds(0.15),
                    EndEffectorCommands.stop(endEffector),
                    IntakeRollersCommands.stop(intakeRollers),
                    AlgaePivotCommands.stow(algaepivot)
                )
            );
            
            // L2
            joystick.b().onTrue(
                Commands.sequence(
                    AlgaePivotCommands.stow(algaepivot),
                    EndEffectorCommands.stop(endEffector),
                    IntakeRollersCommands.stop(intakeRollers),
                    SensorRangeCommands.waitForCoral(eeCoral, 0, 100),
                    new InstantCommand(() -> {
                        selectedMode = RobotMode.SCORE_L2;
                        modeStep = 0;
                        System.out.println("Mode set to L2-");
                    }),
                    ElevatorCommands.goToLevel2(elevator)
                )
            );

            //L3
            joystick.x().onTrue(
                Commands.sequence(
                    AlgaePivotCommands.stow(algaepivot),
                    EndEffectorCommands.stop(endEffector),
                    IntakeRollersCommands.stop(intakeRollers),
                    SensorRangeCommands.waitForCoral(eeCoral, 0, 100),
                    new InstantCommand(() -> {
                        selectedMode = RobotMode.SCORE_L3;
                        modeStep = 0;
                        System.out.println("Mode set to L3-");
                    }),
                    ElevatorCommands.goToLevel3(elevator)
                )
            );

            //L4
            joystick.y().onTrue(
                Commands.sequence(
                    AlgaePivotCommands.stow(algaepivot),
                    EndEffectorCommands.stop(endEffector),
                    IntakeRollersCommands.stop(intakeRollers),
                    SensorRangeCommands.waitForCoral(eeCoral, 0, 100),
                    new InstantCommand(() -> {
                        selectedMode = RobotMode.SCORE_L4;
                        modeStep = 0;
                        System.out.println("Mode set to L4-");
                    }),
                    ElevatorCommands.goToLevel4(elevator)
                )
            );

            //L1
            joystick.a().onTrue(
                Commands.sequence(
                    AlgaePivotCommands.stow(algaepivot),
                    EndEffectorCommands.stop(endEffector),
                    IntakeRollersCommands.stop(intakeRollers),
                    SensorRangeCommands.waitForCoral(eeCoral, 0, 100),
                    new InstantCommand(() -> {
                        selectedMode = RobotMode.SCORE_L1;
                        modeStep = 0;
                        System.out.println("Mode set to L1-");
                    }),
                    ElevatorCommands.goToLevel1(elevator),
                    AlgaePivotCommands.level1(algaepivot)
                )
            );

            // D-Pad Up = Elevator + Intake sequence
            joystick.povUp().onTrue(
                Commands.sequence(
                    AlgaePivotCommands.stow(algaepivot),
                    EndEffectorCommands.stop(endEffector),
                    IntakeRollersCommands.stop(intakeRollers),
                    new InstantCommand(() -> {
                        selectedMode = RobotMode.ALGAE1;
                        modeStep = 0;
                        System.out.println("Mode set to ALGAE1-");
                    })
                )
            );

            joystick.povRight().onTrue(
                Commands.sequence(
                    AlgaePivotCommands.stow(algaepivot),
                    EndEffectorCommands.stop(endEffector),
                    IntakeRollersCommands.stop(intakeRollers),
                    new InstantCommand(() -> {
                        selectedMode = RobotMode.ALGAE2;
                        modeStep = 0;
                        System.out.println("Mode set to ALGAE2-");
                    })
                )
            );

            joystick.povLeft().onTrue(
                Commands.sequence(
                    AlgaePivotCommands.runEE(algaepivot),
                    EndEffectorCommands.run(endEffector, 0.08),
                    IntakeRollersCommands.stop(intakeRollers),
                    new InstantCommand(() -> {
                        selectedMode = RobotMode.BARGE;
                        modeStep = 0;
                        System.out.println("Mode set to BARGE-");
                    }),
                    ElevatorCommands.goBarge(elevator)

                )
            );

            joystick.povDown().onTrue(
                Commands.sequence(
                    AlgaePivotCommands.stow(algaepivot),
                    EndEffectorCommands.stop(endEffector),
                    IntakeRollersCommands.stop(intakeRollers),
                    new InstantCommand(() -> {
                        selectedMode = RobotMode.GROUNDALGAE;
                        modeStep = 0;
                        System.out.println("Mode set to GROUNDALGAE-");
                    }),
                    EndEffectorCommands.run(endEffector, 0.08)
                )
            );

            // Run RightTriggerx
            joystick.rightTrigger().onTrue(
                new InstantCommand(() -> {
                    switch (selectedMode) {
                        case SCORE_L1 -> {
                            Commands.sequence(
                                EndEffectorCommands.run(endEffector, 0.4),
                                Commands.waitSeconds(1),
                                EndEffectorCommands.stop(endEffector),
                                AlgaePivotCommands.stow(algaepivot),
                                ElevatorCommands.reset(elevator)
                            ).schedule();
                            modeStep = 0;
                            break;
                        }
                        case SCORE_L2 -> {
                            Commands.sequence(
                                AlgaePivotCommands.runEE(algaepivot),
                                Commands.waitSeconds(0.1),
                                EndEffectorCommands.run(endEffector, 0.6),
                                Commands.waitSeconds(0.8),
                                EndEffectorCommands.stop(endEffector),
                                AlgaePivotCommands.stow(algaepivot),
                                ElevatorCommands.reset(elevator)
                            ).schedule();
                            modeStep = 0;
                            break;
                        }
                        case SCORE_L3 -> {
                            Commands.sequence(
                                AlgaePivotCommands.runEE(algaepivot),
                                Commands.waitSeconds(0.1),
                                EndEffectorCommands.run(endEffector, 0.6),
                                Commands.waitSeconds(0.8),
                                EndEffectorCommands.stop(endEffector),
                                AlgaePivotCommands.stow(algaepivot),
                                ElevatorCommands.reset(elevator)
                            ).schedule();
                            modeStep = 0;
                            break;
                        }
                        case SCORE_L4 -> {
                            Commands.sequence(
                                AlgaePivotCommands.runEE(algaepivot),
                                Commands.waitSeconds(0.1),
                                EndEffectorCommands.run(endEffector, 0.6),
                                Commands.waitSeconds(0.8),
                                EndEffectorCommands.stop(endEffector),
                                AlgaePivotCommands.stow(algaepivot),
                                ElevatorCommands.reset(elevator)
                            ).schedule();
                            modeStep = 0;
                            break;
                        }

                        case BARGE -> {
                            Commands.sequence(
                                AlgaePivotCommands.runEE(algaepivot),
                                IntakeRollersCommands.stop(intakeRollers),
                                EndEffectorCommands.run(endEffector, -0.4),
                                Commands.waitSeconds(2),
                                EndEffectorCommands.stop(endEffector),
                                AlgaePivotCommands.stow(algaepivot),
                                ElevatorCommands.reset(elevator)
                            ).schedule();
                            modeStep = 0;
                            break;
                        }

                        // case GROUNDALGAE -> {
                        //     switch (modeStep) {
                        //       case 0 -> {
                        //         Commands.sequence(
                        //             ElevatorCommands.AlgaeGround(elevator),
                        //             EndEffectorCommands.run(endEffector, 0.6),
                        //             AlgaePivotCommands.algaeground(algaepivot)
                        //         ).schedule();
                        //         modeStep = 1;
                        //       }

                        //       case 1 -> {
                        //         Commands.sequence(
                        //             AlgaePivotCommands.stow(algaepivot),
                        //             ElevatorCommands.reset(elevator),
                        //             EndEffectorCommands.run(endEffector, 0.08)
                        //         ).schedule();
                        //         modeStep = 2;
                        //       }

                        //       case 2 -> {
                        //         Commands.sequence(
                        //             ElevatorCommands.reset(elevator),
                        //             AlgaePivotCommands.stow(algaepivot),
                        //             IntakeRollersCommands.stop(intakeRollers),
                        //             EndEffectorCommands.run(endEffector, -0.3),
                        //             Commands.waitSeconds(1),
                        //             EndEffectorCommands.stop(endEffector)
                        //         ).schedule();
                        //         modeStep = 0; // reset steps
                        //       }
                        //       default -> modeStep = 0;
                        //     }
                        // }

                        case ALGAE1 -> {
                            switch (modeStep) {
                              case 0 -> {
                                Commands.sequence(
                                    ElevatorCommands.Algae1(elevator),
                                    AlgaePivotCommands.algae(algaepivot),
                                    EndEffectorCommands.run(endEffector, 0.6),
                                    Commands.waitSeconds(2),
                                    ElevatorCommands.reset(elevator),
                                    AlgaePivotCommands.runEE(algaepivot),
                                    EndEffectorCommands.run(endEffector, 0.08)
                                ).schedule();
                                modeStep = 1;
                              }
                              case 1 -> {
                                Commands.sequence(
                                    AlgaePivotCommands.runEE(algaepivot),
                                    IntakeRollersCommands.stop(intakeRollers),
                                    EndEffectorCommands.run(endEffector, -0.3),
                                    Commands.waitSeconds(1),
                                    EndEffectorCommands.stop(endEffector),
                                    AlgaePivotCommands.stow(algaepivot)
                                ).schedule();
                                modeStep = 0; // reset steps
                              }
                              default -> modeStep = 0;
                            }
                        }

                        case ALGAE2 -> {
                            switch (modeStep) {
                              case 0 -> {
                                Commands.sequence(
                                    ElevatorCommands.Algae2(elevator),
                                    Commands.waitSeconds(1),
                                    AlgaePivotCommands.algae(algaepivot),
                                    EndEffectorCommands.run(endEffector, 0.6),
                                    Commands.waitSeconds(1.5),
                                    ElevatorCommands.reset(elevator),
                                    AlgaePivotCommands.runEE(algaepivot),
                                    EndEffectorCommands.run(endEffector, 0.08)
                                ).schedule();
                                modeStep = 1;
                              }
                              case 1 -> {
                                Commands.sequence(
                                    AlgaePivotCommands.runEE(algaepivot),
                                    IntakeRollersCommands.stop(intakeRollers),
                                    EndEffectorCommands.run(endEffector, -0.3),
                                    Commands.waitSeconds(1),
                                    EndEffectorCommands.stop(endEffector),
                                    AlgaePivotCommands.stow(algaepivot)
                                ).schedule();
                                modeStep = 0; // reset steps
                              }                              
                              default -> modeStep = 0;
                            }
                        }
                        default -> {
                            System.out.println("No mode selected!");
                            break;
                        }
                    }
                })
            );
        }

        public Command getAutonomousCommand() {     
            return autoChooser.getSelected();
        }
    }
