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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ProfiledExternalPIDController;

public class ArmSubsystem extends SubsystemBase {

    private final double ARM0_GEAR_REDUCTION = 60.0;
    private final double ARM0_POS_CONVERSION = 2 * Math.PI / ARM0_GEAR_REDUCTION;
    private final double ARM0_VEL_CONVERSION = ARM0_POS_CONVERSION / 60.0; // Convert meters/minute to meters/second

    private final String m_tuningTable = "Arm/Arm0Tuning";
    private final String m_dataTable = "Arm/Arm0Data";

    private CANSparkMax m_motor0;
    private SparkMaxPIDController m_sparkPid0;
    private RelativeEncoder m_encoder0;

    private ProfiledExternalPIDController m_pid0;

    private DoubleEntry[] m_arm0PIDSubs;
    private DoublePublisher m_arm0PosPub;
    private DoublePublisher m_arm0SetpointPub;
    private DoublePublisher m_arm0VelPub;
    private DoubleEntry m_arm0VoltsAtHorizontal;
    private DoublePublisher m_arm0FFTestingVolts;

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
        m_motor0.setSmartCurrentLimit(40);
        // m_motor0.setClosedLoopRampRate(2);

        m_encoder0 = m_motor0.getEncoder();
        m_encoder0.setPositionConversionFactor(ARM0_POS_CONVERSION);
        m_encoder0.setVelocityConversionFactor(ARM0_VEL_CONVERSION);

        // Lighter Weight
        // double ff = 0;
        // double p = 0.7;
        // double i = 0.001;
        // double d = 0.1;
        // double iZone = 0.08;

        // Heavier Weight
        double ff = 0;
        double p = 1.0;
        double i = 0.001;
        double d = 0.1;
        double iZone = 0.08;

        NetworkTable arm0TuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTable);
        m_arm0PIDSubs = new DoubleEntry[]{
            arm0TuningTable.getDoubleTopic("FF").getEntry(ff),
            arm0TuningTable.getDoubleTopic("P").getEntry(p),
            arm0TuningTable.getDoubleTopic("I").getEntry(i),
            arm0TuningTable.getDoubleTopic("D").getEntry(d),
            arm0TuningTable.getDoubleTopic("IZone").getEntry(iZone),
        };

        if (m_arm0PIDSubs[0].getAtomic().timestamp == 0) {
            m_arm0PIDSubs[0].accept(ff);
            m_arm0PIDSubs[1].accept(p);
            m_arm0PIDSubs[2].accept(i);
            m_arm0PIDSubs[3].accept(d);
            m_arm0PIDSubs[4].accept(iZone);
        }
        
        m_sparkPid0 = m_motor0.getPIDController();
        updatePIDSettings();  
        m_pid0 = new ProfiledExternalPIDController(new Constraints(0, 0));
        setConstraints(true);
        
        NetworkTable arm0DataTable = NetworkTableInstance.getDefault().getTable(m_dataTable);
        m_arm0PosPub = arm0DataTable.getDoubleTopic("MeasuredAngle").publish(PubSubOption.periodic(0.02));
        m_arm0SetpointPub = arm0DataTable.getDoubleTopic("SetpointAngle").publish(PubSubOption.periodic(0.02));
        m_arm0VelPub = arm0DataTable.getDoubleTopic("Vel").publish(PubSubOption.periodic(0.02));
        m_arm0VoltsAtHorizontal = arm0DataTable.getDoubleTopic("VoltsAtHorizontal").getEntry(0);
        m_arm0VoltsAtHorizontal.accept(0.9);

        m_arm0FFTestingVolts = arm0DataTable.getDoubleTopic("VoltageSetInFFTesting").publish();
    }

    public void updatePIDSettings() {
        m_sparkPid0.setFF(m_arm0PIDSubs[0].get());
        m_sparkPid0.setP(m_arm0PIDSubs[1].get());
        m_sparkPid0.setI(m_arm0PIDSubs[2].get());
        m_sparkPid0.setD(m_arm0PIDSubs[3].get());
        m_sparkPid0.setIZone(m_arm0PIDSubs[4].get()); 
    }

    @Override
    public void periodic() {
        m_arm0PosPub.accept(Math.toDegrees(m_encoder0.getPosition()));
        m_arm0VelPub.accept(m_encoder0.getVelocity());
        if (m_encoder0.getVelocity() > 2) {
            m_pid0.setConstraints(new Constraints(2.2, Math.PI * 0.9));
        }
    }

    public void setConstraints(boolean slowerAcceleration) {
        if (slowerAcceleration) {
            m_pid0.setConstraints(new Constraints(2.2, Math.PI * 0.9));
        } else {
            m_pid0.setConstraints(new Constraints(2.2, Math.PI * 1.5));
        }
    }

    public void resetMotionProfile() {
        m_pid0.reset(m_encoder0.getPosition(), m_encoder0.getVelocity());
    }

    public void setArm0(Rotation2d angle) {
        double pidSetpoint = m_pid0.getPIDSetpoint(angle.getRadians());
        m_sparkPid0.setReference(pidSetpoint, ControlType.kPosition, 0, calculateArm0FF());
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
        double voltage = additionalVoltage + calculateArm0FF();
        m_sparkPid0.setReference(voltage, ControlType.kVoltage);
        m_arm0FFTestingVolts.accept(voltage);
    }
}
