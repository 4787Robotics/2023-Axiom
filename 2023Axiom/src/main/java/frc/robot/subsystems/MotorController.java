package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
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
    m_arm.setIdleMode(IdleMode.kBrake);
    m_left_wheel.setNeutralMode(NeutralMode.Brake);
    m_right_wheel.setNeutralMode(NeutralMode.Brake);
    m_clamp.setNeutralMode(NeutralMode.Brake);
  

     // For encoder
     //I have no idea how to make one for SparkMax
    m_left_wheel.setSelectedSensorPosition(0);
    m_right_wheel.setSelectedSensorPosition(0);

    // limits acceleration, takes 0.4 seconds to accelerate from 0 to 100%
    m_arm.setOpenLoopRampRate(0.4); //no idea if this one works
    m_left_wheel.configOpenloopRamp(0.4); 
    m_right_wheel.configOpenloopRamp(0.4); // may need change in the future when robot is active
}

  public void clamp() {
     //Unused for now
  }
}

/**Changes needed but can't be done rn:
 * 
 * Figure out the motors for the clamp and wheels.
 * Install the new libraries for each new type of motor.
 * Use the motor types for the braking, Accel, etc.
 *
 * Add a following version of m_arm when for each new arm motor.
 * 
*/
