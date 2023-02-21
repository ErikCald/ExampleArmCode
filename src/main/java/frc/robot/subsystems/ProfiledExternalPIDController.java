// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

/**
 * Implements a PID control loop whose setpoint is constrained by a trapezoid profile. Users should
 * call reset() when they first start running the controller to avoid unwanted behavior.
 */
public class ProfiledExternalPIDController {
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.Constraints m_constraints;

  private double m_period;

  /**
   * Allocates a ProfiledExternalPIDController with the given constants for Kp, Ki, and Kd.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param constraints Velocity and acceleration constraints for goal.
   */
  public ProfiledExternalPIDController(TrapezoidProfile.Constraints constraints) {
    this(constraints, 0.02);
  }

  /**
   * Allocates a ProfiledExternalPIDController with the given constants for Kp, Ki, and Kd.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param constraints Velocity and acceleration constraints for goal.
   * @param period The period between controller updates in seconds. The default is 0.02 seconds.
   */
  public ProfiledExternalPIDController(TrapezoidProfile.Constraints constraints, double period) {
    m_constraints = constraints;
    m_period = period;
  }

  /**
   * Sets the goal for the ProfiledExternalPIDController.
   *
   * @param goal The desired goal state.
   */
  public void setGoal(TrapezoidProfile.State goal) {
    m_goal = goal;
  }

  /**
   * Sets the goal for the ProfiledExternalPIDController.
   *
   * @param goal The desired goal position.
   */
  public void setGoal(double goal) {
    m_goal = new TrapezoidProfile.State(goal, 0);
  }

  /**
   * Gets the goal for the ProfiledExternalPIDController.
   *
   * @return The goal.
   */
  public TrapezoidProfile.State getGoal() {
    return m_goal;
  }

  /**
   * Set velocity and acceleration constraints for goal.
   *
   * @param constraints Velocity and acceleration constraints for goal.
   */
  public void setConstraints(TrapezoidProfile.Constraints constraints) {
    m_constraints = constraints;
  }

  /**
   * Returns the current setpoint of the ProfiledExternalPIDController.
   *
   * @return The current setpoint.
   */
  public TrapezoidProfile.State getSetpoint() {
    return m_setpoint;
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @return The controller's next output.
   */
  public double getPIDSetpoint() {
    var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
    m_setpoint = profile.calculate(m_period);
    return m_setpoint.position;
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @param goal The new goal of the controller.
   * @return The controller's next output.
   */
  public double getPIDSetpoint(TrapezoidProfile.State goal) {
    setGoal(goal);
    return getPIDSetpoint();
  }

  /**
   * Returns the next output of the PIDController.
   *
   * @param measurement The current measurement of the process variable.
   * @param goal The new goal of the controller.
   * @return The controller's next output.
   */
  public double getPIDSetpoint( double goal) {
    setGoal(goal);
    return getPIDSetpoint();
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @param goal The new goal of the controller.
   * @param constraints Velocity and acceleration constraints for goal.
   * @return The controller's next output.
   */
  public double getPIDSetpoint(TrapezoidProfile.State goal, TrapezoidProfile.Constraints constraints) {
    setConstraints(constraints);
    return getPIDSetpoint(goal);
  }

  /**
   * Reset the previous error and the integral term.
   *
   * @param measurement The current measured State of the system.
   */
  public void reset(TrapezoidProfile.State measurement) {
    m_setpoint = measurement;
  }

  /**
   * Reset the previous error and the integral term.
   *
   * @param measuredPosition The current measured position of the system.
   * @param measuredVelocity The current measured velocity of the system.
   */
  public void reset(double measuredPosition, double measuredVelocity) {
    reset(new TrapezoidProfile.State(measuredPosition, measuredVelocity));
  }

  /**
   * Reset the previous error and the integral term.
   *
   * @param measuredPosition The current measured position of the system. The velocity is assumed to
   *     be zero.
   */
  public void reset(double measuredPosition) {
    reset(measuredPosition, 0.0);
  }
}
