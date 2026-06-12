package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.IntakeConstants.PivotSetPoints;
import frc.robot.Constants.canIDs;

public class PivotSubsystem extends SubsystemBase {

  private TalonFXS m_otorCont;

  private RelativeEncoder re_pivotMotor;
  private AbsoluteEncoder ae_pivotMotor;

  private SparkClosedLoopController p_pivotMotor;

  public PivotSubsystem() {
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */

    m_otorCont.getConfigurator().apply(IntakeConfigs.pivotConfig);

    p_pivotMotor = m_pivotMotor.getClosedLoopController();

    re_pivotMotor = m_pivotMotor.getEncoder();
    ae_pivotMotor = m_pivotMotor.getAbsoluteEncoder();

    setTargetPosition(
        PivotSetPoints.kStartPosition); // set target position to start position and go there
  }

  public Command setTargetPosition(double setpos) {
    return runOnce(
        () ->
            p_pivotMotor.setSetpoint(
                setpos, ControlType.kMAXMotionPositionControl) // USING PID POSITION CONTROL
        );
  }

  public Command clearFaults() {
    return runOnce(() -> m_pivotMotor.clearFaults());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot/" + "Output", m_pivotMotor.getAppliedOutput());
    SmartDashboard.putNumber("Pivot/" + "Current", m_pivotMotor.getOutputCurrent());
    SmartDashboard.putNumber("Pivot/" + "Relative/" + "Rel Position", re_pivotMotor.getPosition());
    SmartDashboard.putNumber("Pivot/" + "Absolute/" + "Abs Position", ae_pivotMotor.getPosition());
    SmartDashboard.putNumber("Pivot/" + "Relative/" + "Rel Velocity", re_pivotMotor.getVelocity());
    SmartDashboard.putNumber("Pivot/" + "Absolute/" + "Abs Velocity", ae_pivotMotor.getVelocity());
  }
}
