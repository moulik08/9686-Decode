package org.firstinspires.ftc.teamcode.teleops;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU.Parameters;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TarunTeleop", group = "Robot")

public class TarunTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {




        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "fL");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "bL");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "fR");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "bR");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;






        while(opModeIsActive()){
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double r = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double roboX =  x * Math.cos(-heading) - y * Math.sin(-heading);
            double roboY = x * Math.sin(-heading) + y * Math.cos(-heading);

            double denominator = Math.max( Math.abs(roboX) + Math.abs(roboY) + Math.abs(r),1);
            roboX= roboX * 1.1;

            double frontLeftPower = (roboY + roboX + r) / denominator;
            double backLeftPower = (roboY - roboX + r) / denominator;
            double frontRightPower =(roboY - roboX - r ) / denominator;
            double backRightPower = (roboY + roboX - r) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

        }

    }
}