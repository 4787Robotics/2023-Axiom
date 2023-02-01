// package frc.robot.subsystems;

// import frc.robot.Constants;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
// import com.ctre.phoenix.motorcontrol.NeutralMode;

// public class MotorController extends SubsystemBase {
//   private WPI_TalonFX m_arm, m_leftGrip, m_rightGrip;
//   // Creates and initializes motor controllers.
//   public MotorController() {
//     m_arm = new WPI_TalonFX(Constants.MOTOR_ARM);
//     m_leftGrip = new WPI_TalonFX(Constants.MOTOR_LEFT_GRIP);
//     m_rightGrip = new WPI_TalonFX(Constants.MOTOR_RIGHT_GRIP);

//     m_leftGrip.setInverted(TalonFXInvertType.Clockwise); // Makes left motor go the opposite of the right motor
    
//     m_arm.setNeutralMode(NeutralMode.Brake); // Keeps the motors in place and stops them from moving without input
//     m_leftGrip.setNeutralMode(NeutralMode.Brake);
//     m_rightGrip.setNeutralMode(NeutralMode.Brake);

//     m_rightGrip.setSelectedSensorPosition(0); // for encoder
//     m_leftGrip.setSelectedSensorPosition(0);

//     m_leftGrip.configOpenloopRamp(0.4); // limits acceleration, takes 0.4 seconds to accelerate from 0 to 100%
//     m_rightGrip.configOpenloopRamp(0.4); // may need change in the future when robot is active
//   }
//}