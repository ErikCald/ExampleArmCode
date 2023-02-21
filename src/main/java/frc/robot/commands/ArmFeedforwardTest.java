// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;

public class ArmFeedforwardTest extends CommandBase {
    private CommandXboxController m_joystick;
    private double m_maxExtraVolts;

    public ArmFeedforwardTest(CommandXboxController joystick, double maxExtraVolts) {
        m_joystick = joystick;
        m_maxExtraVolts = maxExtraVolts;

        addRequirements(ArmSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ArmSubsystem.getInstance().setArm0IdleMode(IdleMode.kCoast);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double joystickValue = m_joystick.getRawAxis(XboxController.Axis.kRightX.value);
        joystickValue = MathUtil.applyDeadband(joystickValue, 0.15);

        ArmSubsystem.getInstance().testFeedforward(
            joystickValue * m_maxExtraVolts
        );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().stopMotors();
        ArmSubsystem.getInstance().setArm0IdleMode(IdleMode.kBrake);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
