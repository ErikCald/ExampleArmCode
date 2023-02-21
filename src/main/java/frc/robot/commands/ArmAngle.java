// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmAngle extends CommandBase {
    private final Rotation2d m_angle;
    private final boolean m_slowerAcceleration;
    
    /** Creates a new ArmAngle. */
    public ArmAngle(double angleDeg, boolean slowerAcceleration) {
        m_angle = Rotation2d.fromDegrees(angleDeg);
        m_slowerAcceleration = slowerAcceleration;
        addRequirements(ArmSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ArmSubsystem.getInstance().setConstraints(m_slowerAcceleration);
        ArmSubsystem.getInstance().resetMotionProfile();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ArmSubsystem.getInstance().setArm0(m_angle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().stopMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
