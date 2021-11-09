package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

public class SubsystemDrivetrain {
    DcMotor leftFront, rightFront, leftBack, rightBack;

    public SubsystemDrivetrain(DcMotor _leftFront, DcMotor _rightFront,
                           DcMotor _leftBack, DcMotor _rightBack) {
        leftFront = _leftFront;
        rightFront = _rightFront;
        leftBack = _leftBack;
        rightBack = _rightBack;

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setDriverPower(Gamepad gp) {
        double speed = Math.hypot(gp.left_stick_x, gp.left_stick_y);
        double angle = Math.atan2(gp.left_stick_y, gp.left_stick_x) - Math.PI/4;
        double turn = gp.right_stick_x;

        final double lfPower = speed * Math.sin(angle) + turn;
        final double rfPower = speed * Math.cos(angle) - turn;
        final double lbPower = speed * Math.cos(angle) + turn;
        final double rbPower = speed * Math.sin(angle) - turn;

        leftFront.setPower(lfPower);
        rightFront.setPower(rfPower);
        leftBack.setPower(lbPower);
        rightBack.setPower(rbPower);
    }
}
