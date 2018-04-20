/* Copyright (c) 2017 FIRST. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted (subject to the limitations in the disclaimer below) provided that
* the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this list
* of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice, this
* list of conditions and the following disclaimer in the documentation and/or
* other materials provided with the distribution.
*
* Neither the name of FIRST nor the names of its contributors may be used to endorse or
* promote products derived from this software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
* LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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
