package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTagDetection", group = "Robot")
public class AprilTagSimpleDetection extends LinearOpMode {

    // Define the position and orientation of the camera on the robot.
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    // Variables for the AprilTag processor and Vision Portal.
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("bR");

        DcMotor flyWheel = hardwareMap.dcMotor.get("fly");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Initialize the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        // Initialize the Vision Portal.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // Display readiness on the Driver Station.
        telemetry.addData("Status", "Camera and AprilTag Processor Ready");
        telemetry.update();

        waitForStart();

        // Main OpMode loop for localization.
        while (!isStopRequested() && opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double flyPower = gamepad2.left_stick_y;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            flyWheel.setPower(flyPower);

            //april tag detection in parallel


            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Iterate through the detections and display relevant localization data.
            for (AprilTagDetection detection : currentDetections) {
                telemetry.addLine(String.format("\n--- Tag ID %d ---", detection.id));
                telemetry.addLine(String.format("Center (pixels): %.0f, %.0f", detection.center.x, detection.center.y));
                
                // Debug information
                telemetry.addLine("Debug Info:");
                telemetry.addLine("ftcPose != null: " + (detection.ftcPose != null));
                telemetry.addLine("metadata != null: " + (detection.metadata != null));
                telemetry.addLine("robotPose != null: " + (detection.robotPose != null));

                // Add null check for ftcPose
                if (detection.ftcPose != null) {
                    telemetry.addLine("FTC Pose Data:");
                    telemetry.addLine(String.format("Distance: %.1f inches, Bearing: %.1f degrees",
                            detection.ftcPose.range, detection.ftcPose.bearing));
                    telemetry.addLine(String.format("Elevation: %.1f degrees", detection.ftcPose.elevation));
                    telemetry.addLine(String.format("X: %.1f, Y: %.1f", detection.ftcPose.x, detection.ftcPose.y));

                    // Check if metadata is available
                    if (detection.metadata != null) {
                        telemetry.addLine(String.format("Name: %s", detection.metadata.name));
                        if (detection.metadata.fieldPosition != null) {
                            telemetry.addLine("Field Position: " + detection.metadata.fieldPosition.toString());
                        } else {
                            telemetry.addLine("Field Position: Not available");
                        }
                    } else {
                        telemetry.addLine("Metadata: Not in library");
                    }
                } else {
                    telemetry.addLine("FTC Pose Data: Not available");
                }

                // Check if robot pose is available
                if (detection.robotPose != null) {
                    telemetry.addLine("Robot Pose Available:");
                    telemetry.addLine(String.format("Robot XYZ (inch): %.1f, %.1f, %.1f",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("Robot PRY (deg): %.1f, %.1f, %.1f",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                } else {
                    telemetry.addLine("Robot Pose: Not available");
                }
            }

            telemetry.update();

            // Pause to reduce CPU usage.
            sleep(20);
        }

        // Close the Vision Portal on OpMode stop.
        visionPortal.close();
    }
}