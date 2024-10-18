// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Climber.ExtendLeft;
import frc.robot.Commands.Climber.ExtendRight;
import frc.robot.Commands.Climber.RetractLeft;
import frc.robot.Commands.Climber.RetractRight;
import frc.robot.Commands.Intake.EmitNote;
import frc.robot.Commands.Intake.IntakeNote;
import frc.robot.Commands.Shooter.ChargeShooter;
import frc.robot.Commands.Shooter.ChargeShooterToAmp;
import frc.robot.Commands.Shooter.ReleaseNote;
import frc.robot.Commands.Shooter.TurnShooterTo;
import frc.robot.Subsystems.Shooter;

public class RobotContainer {
  private static final int CHASSIS_PORT = 0;
  private static final int OPERATOR_PORT = 1;

   //controllers
  public static final CommandXboxController chassis = new CommandXboxController(CHASSIS_PORT);
  public static final CommandXboxController operator = new CommandXboxController(OPERATOR_PORT);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    //intake
    operator.a().whileTrue(new IntakeNote());
    operator.b().onTrue(new TurnShooterTo(98));
    operator.back().whileTrue(new EmitNote());
    operator.y().whileTrue(new ChargeShooterToAmp());
    operator.x().whileTrue(new ReleaseNote());

    //climber
    operator.rightBumper().whileTrue(new ExtendRight());//RB
    operator.leftBumper().whileTrue(new ExtendLeft());//LB
    operator.rightTrigger().whileTrue(new RetractRight());//RT
    operator.leftTrigger().whileTrue(new RetractLeft());//RL

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
