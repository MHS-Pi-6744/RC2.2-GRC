package frc.robot.motor_ctl;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import com.ctre.phoenix6.hardware.TalonFXS;

public class TalonFXSMotorController extends SubsystemBase {
    public static TalonFXS motor;

    public TalonFXSMotorController(int canID, TalonFXSConfiguration config) {
        System.out.println("Talon Subsystem Initialized");
        motor = new TalonFXS(canID);
        
        motor.getConfigurator().apply(config);
    }

    public void magicDuty(double position) {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        motor.setControl(m_request.withPosition(position));
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    /**
     * @apiNote Use an output of -1.0 to 1.0
     */
    public Command M_Move(double output) {
        return runOnce(
            () -> motor.setControl(new DutyCycleOut(output))
        );
    }

    public Command clearFaults() {
        return runOnce(() -> motor.clearStickyFaults());
    }
}
