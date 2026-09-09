package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Rumble")
public class Rumble extends LinearOpMode {


    private static final double DRIVE_SPEED_LIMIT = 0.8;
    private static final double BLADE_RPM = 6000;
    private static final double HEADING_P = 0.05;
    // ==========================================

    private IMU imu;

    @Override
    public void runOpMode() {
        // --- HARDWARE MAPPING ---
        DcMotorEx leftDrive = hardwareMap.get(DcMotorEx.class, "leftDrive");
        DcMotorEx rightDrive = hardwareMap.get(DcMotorEx.class, "rightDrive");
        DcMotorEx bladeMotor = hardwareMap.get(DcMotorEx.class, "bladeMotor");

        // --- IMPORT DcMotorSimple: Set Directions ---
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- ENCODERS ---
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bladeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bladeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // ==========================================

        // --- IMU INIT ---
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        // --- BOOLEAN LOCK FOR BLADES ---
        boolean bladeRunning = false;

        waitForStart();

        while (opModeIsActive()) {


            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;


            double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double headingError = currentYaw;
            double correction = headingError * HEADING_P;
            turn = turn + correction;

            // 3. DRIVETRAIN AT 0.8 POWER
            double leftPower = (drive + turn) * DRIVE_SPEED_LIMIT;
            double rightPower = (drive - turn) * DRIVE_SPEED_LIMIT;

            leftPower = Range.clip(leftPower, -DRIVE_SPEED_LIMIT, DRIVE_SPEED_LIMIT);
            rightPower = Range.clip(rightPower, -DRIVE_SPEED_LIMIT, DRIVE_SPEED_LIMIT);

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);


            if (gamepad1.y) {
                bladeRunning = true;
            }
            if (gamepad1.x) {
                bladeRunning = false;
            }

            if (bladeRunning) {
                bladeMotor.setVelocity(6000, AngleUnit.DEGREES); 
            } else {
                bladeMotor.setPower(0.0);
            }

            telemetry.addData("Yaw (Degrees)", "%.2f", currentYaw);
            telemetry.addData("Left Speed (ticks/sec)", leftDrive.getVelocity());
            telemetry.addData("Right Speed (ticks/sec)", rightDrive.getVelocity());
            telemetry.addData("Blade State", bladeRunning ? "ON (" + BLADE_RPM + " RPM)" : "OFF");
            telemetry.addData("Drive Limit", DRIVE_SPEED_LIMIT);
            telemetry.addData("Zero Power Mode", "BRAKE (Safety Engaged)");
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        bladeMotor.setPower(0);
    }
}