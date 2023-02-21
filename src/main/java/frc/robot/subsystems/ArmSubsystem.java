// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private final double ARM0_GEAR_REDUCTION = 60.0;
    private final double ARM0_POS_CONVERSION = 2 * Math.PI / ARM0_GEAR_REDUCTION;
    private final double ARM0_VEL_CONVERSION = ARM0_POS_CONVERSION / 60.0; // Convert meters/minute to meters/second

    private final String m_tuningTable = "Arm/Arm0Tuning";
    private final String m_dataTable = "Arm/Arm0Data";

    private CANSparkMax m_motor0;
    private SparkMaxPIDController m_sparkPid0;
    private RelativeEncoder m_encoder0;

    private ProfiledPIDController m_pid0;
    private ArmFeedforward m_arm0FF;
    private double lastSpeed = 0;
    private double lastTime = 0;


    private DoubleEntry[] m_arm0PIDSubs;
    private DoublePublisher m_arm0PosPub;
    private DoublePublisher m_arm0SetpointPub;
    private DoubleEntry m_arm0VoltsAtHorizontal;


    private static ArmSubsystem INSTANCE;
    public static ArmSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ArmSubsystem();
        }
        return INSTANCE;
    }

    /** Creates a new ArmSubsystem. */
    private ArmSubsystem() {
        setupArm0();
       
    }

    private void setupArm0() {
        m_motor0 = new CANSparkMax(18, MotorType.kBrushless);
        m_motor0.restoreFactoryDefaults();
        m_motor0.setInverted(false);
        m_motor0.setIdleMode(IdleMode.kBrake);
        m_motor0.setSmartCurrentLimit(35);
        // m_motor0.setClosedLoopRampRate(2);

        m_encoder0 = m_motor0.getEncoder();
        m_encoder0.setPositionConversionFactor(ARM0_POS_CONVERSION);
        m_encoder0.setVelocityConversionFactor(ARM0_VEL_CONVERSION);

        NetworkTable arm0TuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTable);
        m_arm0PIDSubs = new DoubleEntry[]{
            arm0TuningTable.getDoubleTopic("FF").getEntry(0.0),
            arm0TuningTable.getDoubleTopic("P").getEntry(0.0),
            arm0TuningTable.getDoubleTopic("I").getEntry(0.0),
            arm0TuningTable.getDoubleTopic("D").getEntry(0.0),
            arm0TuningTable.getDoubleTopic("IZone").getEntry(0.0),
        };

        if (m_arm0PIDSubs[0].getAtomic().timestamp == 0) {
            m_arm0PIDSubs[0].accept(0);
            m_arm0PIDSubs[1].accept(0.0);
            m_arm0PIDSubs[2].accept(0.0);
            m_arm0PIDSubs[3].accept(0.0);
            m_arm0PIDSubs[4].accept(0.0);
        }
        
        m_sparkPid0 = m_motor0.getPIDController();
        m_pid0 = new ProfiledPIDController(0, 0, 0, new Constraints(Math.PI/4, Math.PI/4));
        updatePIDSettings();  

        m_arm0FF = new ArmFeedforward(0, 0, 0, 0);
        
        NetworkTable arm0DataTable = NetworkTableInstance.getDefault().getTable(m_dataTable);
        m_arm0PosPub = arm0DataTable.getDoubleTopic("MeasuredAngle").publish(PubSubOption.periodic(0.02));
        m_arm0SetpointPub = arm0DataTable.getDoubleTopic("SetpointAngle").publish(PubSubOption.periodic(0.02));
        m_arm0VoltsAtHorizontal = arm0DataTable.getDoubleTopic("VoltsAtHorizontal").getEntry(0);
        m_arm0VoltsAtHorizontal.accept(0.5);
    }

    public void updatePIDSettings() {
        m_pid0.setPID(
            m_arm0PIDSubs[1].get(), m_arm0PIDSubs[2].get(), m_arm0PIDSubs[3].get());

    }

    @Override
    public void periodic() {
        m_arm0PosPub.accept(Math.toDegrees(m_encoder0.getPosition()));
    }

    public void setArm0(Rotation2d angle) {
        double pidVal = m_pid0.calculate(m_encoder0.getPosition(), angle.getRadians());
        double acceleration = (m_pid0.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
        m_motor0.setVoltage(
            pidVal
            + m_arm0FF.calculate(m_pid0.getSetpoint().velocity, acceleration));
        lastSpeed = m_pid0.getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();

        m_arm0SetpointPub.accept(angle.getDegrees());
    }

    private double calculateArm0FF() {
        return m_arm0VoltsAtHorizontal.get() * Math.cos(m_encoder0.getPosition());
    }

    public void resetArm0Encoder(double valueDeg) {
        m_encoder0.setPosition(Math.toRadians(valueDeg));
    }

    public void setArm0IdleMode(IdleMode mode) {
        m_motor0.setIdleMode(mode);
    }

    public void stopMotors() {
        m_motor0.stopMotor();
    }

    public void testFeedforward(double additionalVoltage) {        
        m_sparkPid0.setReference(additionalVoltage + calculateArm0FF(), ControlType.kVoltage);
    }
}
