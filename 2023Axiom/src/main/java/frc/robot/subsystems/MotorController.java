package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class WMotorController extends SubsystemBase {
  // Big Arm Motor
  private CANSparkMax m_arm = new CANSparkMax(Constants.MOTOR_ARM, MotorType.kBrushless);
  // Lower Arm clamp motor
  private WPI_TalonFX m_clamp = new WPI_TalonFX(Constants.MOTOR_CLAMP);

  // Controls wheels that suck em up
  private WPI_TalonFX m_left_wheel = new WPI_TalonFX(Constants.MOTOR_LEFT_WHEEL);
  private WPI_TalonFX m_right_wheel = new WPI_TalonFX(Constants.MOTOR_RIGHT_WHEEL);

  public WMotorController(double[] voltages) {
    if (voltages.length != 4) {
      System.out.println("Need 4 arguments");
      return;
    }
    // Makes left motor go the opposite of the right motor
    m_left_wheel.setInverted(TalonFXInvertType.Clockwise); 
     // ewqeKeeps the motors in place and stops them frowm moving without input
    m_arm.setNeutralMode(NeutralMode.Brake);
    m_left_wheel.setNeutralMode(NeutralMode.Brake);
    m_right_wheel.setNeutralMode(NeutralMode.Brake);
    m_clamp.setNeutralMode(NeutralMode.Brake);
  

     // For encoder
    m_left_wheel.setSelectedSensorPosition(0);
    m_right_wheel.setSelectedSensorPosition(0);

    // limits acceleration, takes 0.4 seconds to accelerate from 0 to 100%
    m_left_wheel.configOpenloopRamp(0.4); 
    m_right_wheel.configOpenloopRamp(0.4); // may need change in the future when robot is active
}

  public void clamp() {
    
  }
}
