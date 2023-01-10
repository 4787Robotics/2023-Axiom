package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class UMotorController extends SubsystemBase {
  // Big Arm Motor
  private m_arm = new WPI_TalonFX(Constants.motor_arm);
  // Lower Arm clamp motor
  private m_clamp = new WPI_TalonFX(Constants.MOTOR_CLAMP);

  // Controls wheels that suck em up
  private m_left_wheel = new WPI_TalonFX(Constants.MOTOR_LEFT_WHEEL);
  private m_right_wheel = new WPI_TalonFX(Constants.MOTOR_RIGHT_WHEEL);

  public UMotorController(double[] voltages) {
    // Makes left motor go the opposite of the right motor
    m_left.setInverted(TalonFXInvertType.Clockewise); 
     // ewqeKeeps the motors in place and stops them frowm moving without input
    m_arm.setNeutralMode(NeutralMode.Brake);
    m_left_wheel.setNeutralMode(NeutralMode.Brake);
    m_right_wheel.setNeutralMode(NeutralMode.Brake);
    m_clamp.setNeutralMode(NeutralMode.Brake
  

     // For encoder
    m_rightGrip.setSelectedSensorPosition(0);
    m_leftGrip.setSelectedSensorPosition(0);

    // limits acceleration, takes 0.4 seconds to accelerate from 0 to 100%
  m_leftGrip.configOpenloopRamp(0.4); 
  m_rightGrip.configOpenloopRamp(0.4); // may need change in the future when robot is active
}

public void clamp() {
  m_leftGrip.
}
}