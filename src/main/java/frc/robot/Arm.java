package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
    XRPServo arm;
    final static double PICKUP = 0.90; // originally 0.85
    final static double CARRY = 0.2; // originally 0.69

    public Arm() {
        super();
        arm = new XRPServo(4);
    }

    @Override
    public void periodic() {

    }

    public Command setAngle(double pos) {
        return run(() -> {
            arm.setPosition(pos);
        }).withName("Arm.setAngle()");
    }

    public Command setPosition(DoubleSupplier pos) {
        return run(() -> {
            //need a debounce on my servo to not grind the end
            var position = MathUtil.clamp(pos.getAsDouble(), 0.1, 0.9);
            SmartDashboard.putNumber("ArmCommand", position);
            arm.setPosition(position);
        }).withName("Arm.setAngle(Supplier)");
    }
}
