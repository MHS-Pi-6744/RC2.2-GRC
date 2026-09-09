package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.IntakeConstants.PivotSetPoints;
import frc.robot.Constants.canIDs;
import frc.robot.motor_ctl.TalonFXSMotorController;

public class PivotSubsystem extends SubsystemBase {
  private TalonFXSMotorController controller =
      new TalonFXSMotorController(canIDs.kPivotMotorCanId, IntakeConfigs.pivotConfig);

  public PivotSubsystem() {
    System.out.println("PivotSubsystem Method Initialized");
    setTargetPosition(PivotSetPoints.kStartPosition / PivotSetPoints.kPositionConversionFactorAbs);
  }

  /**
   * @apiNote this uses Deg
   */
  public Command setTargetPosition(double pos) {
    return runOnce(() -> controller.M_Move(pos / PivotSetPoints.kPositionConversionFactorAbs));
  }

  public Command clearFaults() {
    return runOnce(() -> controller.clearFaults());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot/" + "Motor Velocity", controller.getVelocity());
    SmartDashboard.putNumber("Pivot/" + "Motor Position", controller.getPosition());
    // Using the Conversion Factor to display a number that is relative to the actual robot position
    SmartDashboard.putNumber(
        "Pivot/" + "Motor Velocity Scaled",
        controller.getVelocity() * PivotSetPoints.kPositionConversionFactorAbs);
    SmartDashboard.putNumber(
        "Pivot/" + "Motor Position Scaled",
        controller.getPosition() * PivotSetPoints.kPositionConversionFactorAbs);
  }
}
