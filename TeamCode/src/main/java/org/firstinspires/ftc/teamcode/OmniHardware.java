package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
* This class initializes all of the robot’s hardware.
*/

public class OmniHardware {
   DcMotor backLeftMotor = null;
   DcMotor backRightMotor = null;
   DcMotor middleMotor = null;
   DcMotor leftMec = null;
   DcMotor rightMec = null;
   DcMotor leftPully = null;
   DcMotor rightPully = null;
   DcMotor horizontalSlide = null;
   ColorSensor colorSensor = null;
   GyroSensor gyroSensor = null;
   //DcMotor lights = null;
   Servo colorServo = null;
   Servo relicGrabber = null;
   Servo relicWrist = null;

   public final double SERVO_UP = 0;
   public final double SERVO_DOWN = 1.0;
   public final double SERVO_MIDDLE = 0.3;
   public final double RELIC_STORAGE = 0, RELIC_GRAB = 0.5, RELIC_OVER = 1.0;
   public final double RELIC_HAND_OPEN = 0, RELIC_HAND_CLOSE = 1;
   public final int NEVEREST_TICKS_PER_REV = 1120;
   public final double OMNI_DIAMETER = 4; // in inches
   public final double OMNI_CIRCUMFERENCE = OMNI_DIAMETER * 3.14; // in inches

   public OmniHardware(HardwareMap hardwareMap) {
      //constructs object based on hardwareMap of opMode
      backLeftMotor = hardwareMap.get(DcMotor.class, "blm");
      backRightMotor = hardwareMap.get(DcMotor.class, "brm");
      middleMotor = hardwareMap.get(DcMotor.class, "mm");
      leftMec = hardwareMap.get(DcMotor.class, "lm");
      rightMec = hardwareMap.get(DcMotor.class, "rm");
      leftPully = hardwareMap.get(DcMotor.class, "lp");
      rightPully = hardwareMap.get(DcMotor.class, "rp");
      horizontalSlide = hardwareMap.get(DcMotor.class, "hs");
      colorSensor = hardwareMap.get(ColorSensor.class, "cs");
      gyroSensor = hardwareMap.get(GyroSensor.class, "gs");
      middleMotor = hardwareMap.get(DcMotor.class, "mm");
      colorServo = hardwareMap.get(Servo.class, "colorservo");
      //lights = hardwareMap.get(DcMotor.class, "lights");
      relicGrabber = hardwareMap.get(Servo.class, "relicgrabber");
      relicWrist = hardwareMap.get(Servo.class, "relicwrist");
   }

   public void initHardware() {
      //called during init() of OpMode
      backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
      backRightMotor.setDirection(DcMotor.Direction.REVERSE);
      middleMotor.setDirection(DcMotor.Direction.REVERSE);
      leftMec.setDirection(DcMotor.Direction.FORWARD);
      rightMec.setDirection(DcMotor.Direction.REVERSE);
      leftPully.setDirection(DcMotor.Direction.FORWARD);
      rightPully.setDirection(DcMotor.Direction.REVERSE);
      //lights.setPower(1.0);
      horizontalSlide.setDirection(DcMotor.Direction.FORWARD);
      colorServo.setPosition(SERVO_UP);
      relicWrist.setPosition(RELIC_OVER);
   }
}
