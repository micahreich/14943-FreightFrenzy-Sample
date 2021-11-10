package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.SubsystemDrivetrain;

// Main teleop program

@TeleOp(name="Sample Mecanum Drive", group = "teleop")
public class SampleMecanumDrive extends LinearOpMode {
    // hardware objects
    DcMotor leftFront, rightFront, leftBack, rightBack;

    // subsystem objects
    SubsystemDrivetrain drive;

    @Override
    public void runOpMode() throws InterruptedException {
        // timers
        ElapsedTime timer = new ElapsedTime();

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        drive = new SubsystemDrivetrain(leftFront, rightFront, leftBack, rightBack);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            // use mecanum drivetrain with gamepad 1
            drive.setDriverPower(gamepad1);

            // telemetry logging -- displays information to driver phone
            telemetry.addData("runtime: ", timer.seconds());
            telemetry.update();
        }
    }
}
