package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.motor_ctl.TalonFXSMotorController;
import frc.robot.Constants.canIDs;
import frc.robot.Constants.IntakeConstants.PivotSetPoints;
import frc.robot.Configs.IntakeConfigs;

public class PivotSubsystem extends SubsystemBase {
	private TalonFXSMotorController controller = new TalonFXSMotorController(canIDs.kPivotMotorCanId, IntakeConfigs.pivotConfig);

	public PivotSubsystem()
	{
		setTargetPosition(PivotSetPoints.kStartPosition);
	}

	public Command setTargetPosition(double pos) {
		return runOnce(
			() -> controller.M_Move(pos)
		);
	}

	public Command clearFaults() {
		return runOnce(
			() -> controller.clearFaults()
		);
	}

	@Override
    public void periodic()
    {
        SmartDashboard.putNumber("Pivot/" + "Motor Velocity", controller.getVelocity());
        SmartDashboard.putNumber("Pivot/" + "Motor Position", controller.getPosition());
    }
}