package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// Main teleop program

@TeleOp(name="Sample TeleOp", group = "teleop")
public class SampleTeleOp extends LinearOpMode {
    // hardware objects
    DcMotor intake1, intake2;
    DcMotor leftFront, rightFront, leftBack, rightBack;
    DcMotorEx shooter1, shooter2;
    Servo flicker1, wobble1, wobble2, wobbleClaw;

    // uncomment the below portion if you want to use the IMU
    /*
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double initialAngle = Math.PI;
    double globalAngle = 0;
    */

    // subsystem objects
    SubsystemDrivetrain drive;
    SubsystemIntake intake;
    SubsystemShooter shooter;
    SubsystemWobbleArm wobbleArm;
    SubsystemFlicker flicker;

    boolean useIMU = true;

    int shooterCounterA = 1;
    int shooterCounterB = 1;
    int psCounter = 1;

    boolean shooterLastStateA = false;
    boolean shooterLastStateB = false;
    boolean psLastState = false;

    int shotCounter = 0;

    boolean armGoingUp = false;
    boolean armGoingDown = false;
    boolean armGoingMid = false;

    double triggerTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // counters, toggles
        SubsystemShooter.SHOOTER_STATE currentState = SubsystemShooter.SHOOTER_STATE.HIGH_GOAL;

        // timers
        ElapsedTime timer = new ElapsedTime();

        // hardware references
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        flicker1 = hardwareMap.get(Servo.class, "flicker");
        wobble1 = hardwareMap.get(Servo.class, "wobble1");
        wobble2 = hardwareMap.get(Servo.class, "wobble2");
        wobbleClaw = hardwareMap.get(Servo.class, "wobbleClaw");



        // uncomment the below portion if you want to use the IMU

        /*
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;


        try {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        } catch (Exception e) {
            useIMU = false;
        }
        */

        // subsystem creation
        intake = new SubsystemIntake(intake1, intake2);
        wobbleArm = new SubsystemWobbleArm(wobble1, wobble2, wobbleClaw, timer);
        flicker = new SubsystemFlicker(flicker1, timer);
        drive = new SubsystemDrivetrain(leftFront, rightFront, leftBack, rightBack);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            // use mecanum drivetrain with gamepad 1
            drive.setDriverPower(gamepad1);

            // example use of gamepad to control subsystem: intake
            if (gamepad1.y) {
                intake.activate(1.0);
            } else if (gamepad1.x) {
                intake.activate(-1.0);
            } else {
                intake.reset();
            }

            // example use of gamepad to control subsystem: wobble arm control
            if (gamepad2.dpad_up) {
                wobbleArm.activateClaw(0.4);
                armGoingUp = true;
                triggerTime = timer.milliseconds();
            } else if(gamepad2.dpad_down) {
                wobbleArm.activate(0.1);
                armGoingDown = true;
                triggerTime = timer.milliseconds();
            } else if(gamepad2.dpad_left) {
                wobbleArm.activate(0.4);
                armGoingMid = true;
                triggerTime = timer.milliseconds();
            }

            double timeDelay = timer.milliseconds() - triggerTime;

            if (armGoingUp && Math.abs(timeDelay) >= 500) {
                wobbleArm.activate(1.0);
                armGoingUp = false;
            } else if(armGoingDown && Math.abs(timeDelay) >= 500) {
                wobbleArm.activateClaw(0.0);
                armGoingDown = false;
            } else if(armGoingMid && Math.abs(timeDelay) >= 500) {
                wobbleArm.activateClaw(0.0);
                armGoingMid = false;
            }

            // telemetry logging -- displays information to driver phone
            telemetry.addData("runtime: ", timer.milliseconds());
            telemetry.update();
        }
    }

    // below method for using IMU -- can ignore for now
    /*
    private double getIMUHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double deltaAngle =  initialAngle + angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -Math.PI) {
            deltaAngle += 2 * Math.PI;
        } else if (deltaAngle > Math.PI) {
            deltaAngle -= 2 * Math.PI;
        }

        return deltaAngle;
    }
     */
}
