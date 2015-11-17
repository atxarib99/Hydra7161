package com.qualcomm.ftcrobotcontroller.opmodes;

import android.media.MediaPlayer;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

/**
 * Created by Joshua on 11/12/2015.
 */
public class TrollBot extends OpMode {
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;
    MediaPlayer song;
    ArrayList<String> encoderValues;
    Servo right;
    Servo left;
    private static final String LOG_TAG = TrollBot.class.getSimpleName();
    private int getEncoderAvg() {
        return (motorBL.getCurrentPosition() + motorBR.getCurrentPosition() + motorFL.getCurrentPosition() + motorFR.getCurrentPosition()) / 4;
    }
    public ArrayList<String> getEncoderValues() {
        return encoderValues;
    }
    public void init() {
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        encoderValues = new ArrayList<String>();
        song = MediaPlayer.create(FtcRobotControllerActivity.appActivity, R.raw.song);
        song.setLooping(true);
        song.start();
        right = hardwareMap.servo.get("right");
        right.setPosition(0);
        left = hardwareMap.servo.get("left");
        left.setPosition(0);
    }
    public void loop() {
        if(Math.abs(gamepad1.right_stick_y) > .05 || Math.abs(gamepad1.left_stick_y) > .05) {
            motorBL.setPower(-gamepad1.left_stick_y);
            motorFL.setPower(-gamepad1.left_stick_y);
            motorBR.setPower(gamepad1.right_stick_y);
            motorFR.setPower(gamepad1.right_stick_y);
        }
        else {
            motorBL.setPower(0);
            motorBR.setPower(0);
            motorFR.setPower(0);
            motorFL.setPower(0);
        }

        telemetry.addData("motorBL", motorBL.getCurrentPosition());

        telemetry.addData("motorBR", motorBR.getCurrentPosition());
        telemetry.addData("motorFL", motorFL.getCurrentPosition());
        telemetry.addData("motorFR", motorFR.getCurrentPosition());
        telemetry.addData("avg", getEncoderAvg());
        if(gamepad1.a) {
            encoderValues.add(getEncoderAvg() + "");
            motorBL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            motorBR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            motorFR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            motorFL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        }
        if(gamepad1.y) {
            String created = "";
            created +=
                    "package com.qualcomm.ftcrobotcontroller.opmodes;\n" +
                    "\n" +
                    "import android.util.Log;\n" +
                    "\n" +
                    "import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;\n" +
                    "import com.qualcomm.robotcore.eventloop.opmode.OpMode;\n" +
                    "import com.qualcomm.robotcore.exception.RobotCoreException;\n" +
                    "import com.qualcomm.robotcore.hardware.ColorSensor;\n" +
                    "import com.qualcomm.robotcore.hardware.DcMotor;\n" +
                    "import com.qualcomm.robotcore.hardware.DcMotorController;\n" +
                    "import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;\n" +
                    "import com.qualcomm.robotcore.hardware.DigitalChannelController;\n" +
                    "import com.qualcomm.robotcore.hardware.GyroSensor;\n" +
                    "import com.qualcomm.robotcore.hardware.LightSensor;\n" +
                    "//import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;\n" +
                    "import com.qualcomm.robotcore.hardware.Servo;\n" +
                    "import com.qualcomm.robotcore.util.ElapsedTime;\n" +
                    "import com.qualcomm.robotcore.robocol.Telemetry;\n" +
                    "\n" +
                    " These are the imports that you need to use you Autonomous/**\n" +
                    " * Autonomous mode\n" +
                    " * enable movement based on sensors and preset code\n" +
                    " */\n" +
                    "public class YOURCLASSNAMEHERE extends LinearOpMode implements hydraDriveBase{\n" +
                    "    //creates motors\n" +
                    "    DcMotor motorBL;\n" +
                    "    DcMotor motorBR;\n" +
                    "    DcMotor motorFL;\n" +
                    "    DcMotor motorFR; //this creates your 4 motors \n" +
                            "    ArrayList<String> encoderValues = new ArrayList<String>;\n";
                    for(int i = 0; i < encoderValues.size(); i++) {
                        created += "encoderValues.add(" + encoderValues.get(i) + ");\n";
                    }

                    created += "    private static final String LOG_TAG = YOURCLASSNAMEHERE.class.getSimpleName(); //This is for logging errors if you encounter any.\n" +
                    "    ElapsedTime elapsedTime;\n" +
                    "    public void startMotors(double power1, double power2, double power3, double power4) { //This is a method that starts your motors \n" +
                    "        motorBR.setPower(power1);\n" +
                    "        motorBL.setPower(power2);\n" +
                    "        motorFL.setPower(power3);\n" +
                    "        motorFR.setPower(power4);\n" +
                    "    }\n" +
                    "    public int getEncoderAvg() { // This gets encoder averages\n" +
                    "        return (Math.abs(motorBL.getCurrentPosition()) + Math.abs(motorBR.getCurrentPosition()) + Math.abs(motorFL.getCurrentPosition()) + Math.abs(motorFR.getCurrentPosition())) / 4;\n" +
                    "    }\n" +
                    "    public void stopMotors() {//This stops all your motors\n" +
                    "        motorBR.setPower(0);\n" +
                    "        motorBL.setPower(0);\n" +
                    "        motorFL.setPower(0);\n" +
                    "        motorFR.setPower(0);\n" +
                    "    }\n" +
                    "    public void resetEncoders() {//This will reset your encoder values on your robot\n" +
                    "        motorBL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);\n" +
                    "        motorBR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);\n" +
                    "        motorFR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);\n" +
                    "        motorFL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);\n" +
                    "    }\n" +
                    "    public void getEncoderValues() { //This gets each encoder values and oututs them to the driver station\n" +
                    "        telemetry.addData(\"motorBL\", motorBL.getCurrentPosition());\n" +
                    "\n" +
                    "        telemetry.addData(\"motorFR\", motorFR.getCurrentPosition());\n" +
                    "\n" +
                    "        telemetry.addData(\"motorBR\", motorBR.getCurrentPosition());\n" +
                    "\n" +
                    "        telemetry.addData(\"motorFL\", motorFL.getCurrentPosition());\n" +
                    "    }\n" +
                    "    public void getTime() { //This returns the time to the driver station\n" +
                    "        telemetry.addData(\"time\", elapsedTime.time());\n" +
                    "    }\n" +
                    "    @Override\n" +
                    "    public void runOpMode() {//This runs the code itself\n" +
                    "        elapsedTime = new ElapsedTime();\n" +
                    "        motorBL = hardwareMap.dcMotor.get(\"motorBL\");\n" +
                    "        motorBR = hardwareMap.dcMotor.get(\"motorBR\");\n" +
                    "        motorFL = hardwareMap.dcMotor.get(\"motorFL\");\n" +
                    "        elapsedTime.startTime();\n" +
                    "        motorFR = hardwareMap.dcMotor.get(\"motorFR\");\n" +
                            "double currEncoder = 0.0;";
                    for(int i = 0; i < encoderValues.size(); i++) {
                        created += "int distance" + i + " = " + encoderValues.get(i) + ";\n";
                    }
                    for (int k =0; k < encoderValues.size(); k++) {
                        if(Integer.parseInt(encoderValues.get(k)) < 0)
                            created += "        while (currEncoder > distance" + k + ") { \n" +
                                "startMotors(1, -1, -1, 1); \n" +
                                "currEncoder = getEncoderAvg(); \n" +
                                "} \n" +
                                "resetEncoders();\n";
                        else {
                            created += "        while (currEncoder < distance" + k + ") { \n" +
                                    "startMotors(-1, 1, 1, -1); \n" +
                                    "currEncoder = getEncoderAvg(); \n" +
                                    "} \n" +
                                    "resetEncoders();\n";
                        }
                        created += "stopMotors();\n";
                    }
                   created +=
                    "        stopMotors();\n" +
                    "        motorBL.close();\n" +
                    "        motorFL.close();\n" +
                    "        motorBR.close();\n" +
                    "        motorFR.close();\n" +
                    "    }\n" +
                    "}";

        }
    }
    public void stop() {

    }
}
