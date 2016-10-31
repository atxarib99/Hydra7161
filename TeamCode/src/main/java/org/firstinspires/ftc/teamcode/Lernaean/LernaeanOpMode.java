package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;

/**
 * Created by Arib on 9/11/2016.
 */
public abstract class LernaeanOpMode extends OpMode {
    DcMotor motorL;
    DcMotor motorR;
    DcMotor manipulator;
    DcMotor shooterR;
    DcMotor shooterL;
    Servo front;
    Servo back;
    Servo activate;
    DeviceInterfaceModule cdim;
    ColorSensor color;
    OpticalDistanceSensor left;
    OpticalDistanceSensor right;

    private final double BACK_OUT = 1;
    private final double BACK_IN = 0;
    private final double FRONT_OUT = 1;
    private final double FRONT_IN = 0;
    private boolean reversed;

    public double shooterPower;

    Runnable startMotorsR = new Runnable() {
        @Override
        public void run() {
            if(reversed) {
                motorL.setPower(gamepad1.left_stick_y);
                motorR.setPower(-gamepad1.right_stick_y);
            } else {
                motorL.setPower(-gamepad1.left_stick_y);
                motorR.setPower(gamepad1.right_stick_y);
            }
        }
    };

    Runnable stopMotorR = new Runnable() {
        @Override
        public void run() {
            motorL.setPower(0);
            motorR.setPower(0);
        }
    };

    Runnable startManiR = new Runnable() {
        @Override
        public void run() {
            manipulator.setPower(1);
        }
    };

    Runnable stopManiR = new Runnable() {
        @Override
        public void run() {
            manipulator.setPower(0);
        }
    };

    Runnable reverseManiR = new Runnable() {
        @Override
        public void run() {
            manipulator.setPower(-1);
        }
    };

    Runnable frontOutR = new Runnable() {
        @Override
        public void run() {
            frontOut();
        }
    };

    Runnable frontInR = new Runnable() {
        @Override
        public void run() {
            frontIn();
        }
    };

    Runnable backOutR = new Runnable() {
        @Override
        public void run() {
            backOut();
        }
    };

    Runnable backInR = new Runnable() {
        @Override
        public void run() {
            backIn();
        }
    };


    Thread driveThread = new Thread(stopMotorR);
    Thread maniThread = new Thread(stopManiR);
    Thread beaconThread;
    Thread sensorThread;
    Thread rpmStabilization;

    @Override
    public void init() {
        shooterPower = 1;
        reversed = false;
        composeTelemetry();
        motorL = hardwareMap.dcMotor.get("L");
        motorR = hardwareMap.dcMotor.get("R");
        manipulator = hardwareMap.dcMotor.get("mani");
        shooterR = hardwareMap.dcMotor.get("sR");
        shooterL = hardwareMap.dcMotor.get("sL");
        back = hardwareMap.servo.get("back");
        front = hardwareMap.servo.get("front");
        activate = hardwareMap.servo.get("active");
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        color = hardwareMap.colorSensor.get("color");
        left = hardwareMap.opticalDistanceSensor.get("odsL");
        right = hardwareMap.opticalDistanceSensor.get("odsR");
        shooterPower = .5;
        frontIn();
        backIn();
    }

    public void startMotors(double ri, double le) {
        if(reversed) {
            motorL.setPower(le);
            motorR.setPower(-ri);
        } else {
            motorL.setPower(-le);
            motorR.setPower(ri);
        }
    }

    public void stopMotors() {
        motorL.setPower(0);
        motorR.setPower(0);
    }

    public void frontOut() {
        front.setPosition(FRONT_OUT);
    }

    public void frontIn() {
        front.setPosition(FRONT_IN);
    }

    public void backOut() {
        back.setPosition(BACK_OUT);
    }

    public void backIn() {
        back.setPosition(BACK_IN);
    }

    public void reverse() {
        reversed = !reversed;
    }

    public void startMani() {
        manipulator.setPower(1);
    }

    public void stopMani() {
        manipulator.setPower(0);
    }

    public void reverseMani() {
        manipulator.setPower(-1);
    }

    public void startShooter() {
        shooterL.setPower(shooterPower);
        shooterR.setPower(-shooterPower);
    }

    public void reverseShooter() {
        shooterL.setPower(-1);
        shooterR.setPower(1);
    }

    public void stopShooter() {
        shooterL.setPower(0);
        shooterR.setPower(0);
    }

    public void activateShooter(boolean active) {
        if(active)
            activate.setPosition(.45);
        else
            activate.setPosition(1);
    }

    public double getRightODS() {
        return right.getLightDetected();
    }

    public double getLeftODS() {
        return left.getLightDetected();
    }

    public double getRawRightODS() {
        return right.getRawLightDetected();
    }

    public double getRawLeftODS() {
        return left.getRawLightDetected();
    }

    public int getEncoderAvg() {
        return (Math.abs(motorR.getCurrentPosition()) + Math.abs(motorL.getCurrentPosition())) / 2;
    }

    public int getRed() {
        return color.red();
    }

    public int getBlue() {
        return color.blue();
    }

    public double getShooterPower() {
        return shooterPower;
    }

    public void composeTelemetry() {

        telemetry.addLine()
                .addData("rightODS", new Func<String>() {
                    @Override public String value() {
                        return "Right: Raw: " + getRawRightODS() + " Normal: " + getRightODS() + "";
                    }
                });

        telemetry.addLine()
                .addData("leftODS", new Func<String>() {
                    @Override public String value() {
                        return "Left: Raw: " + getRawLeftODS() + " Normal: " + getLeftODS() + "";
                    }
                });

        telemetry.addLine()
                .addData("encoder", new Func<String>() {
                    @Override public String value() {
                        return getEncoderAvg() + "";
                    }
                });
        telemetry.addLine()
                .addData("Color", new Func<String>() {
                    @Override public String value() {
                        return "Red: " + getRed() + " Blue: " + getBlue();
                    }
                });
        telemetry.addLine()
                .addData("ShooterPower", new Func<String>() {
                    @Override public String value() {
                        return "ShooterPower: " + getShooterPower();
                    }
                });


    }



}
