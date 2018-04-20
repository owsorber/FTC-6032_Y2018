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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.ArrayList;

/**
* This class performs all necessary Tele-Op controls.
*/

@TeleOp(name="Omni TeleOp", group="Iterative Opmode")
public class OmniTeleOp extends OpMode {
   OmniHardware hardware;

   // Power variables
   private double leftPower,
         rightPower,
         pullyPower,
         middlePower,
         mecanumPower,
         relicArmPower,
         boost;

   public void init() {
      // Initialize hardware
      hardware = new OmniHardware(hardwareMap);
      hardware.initHardware();

      // Set boost to default
      boost = 0.7;
      middlePower = 0;
      leftPower = 0;
      rightPower = 0;
      hardware.colorServo.setPosition(hardware.SERVO_MIDDLE);
   }

   public void loop() {
      // Call basic Tele-Op methods
      manageDriveTrain();
      applyBoost();
      grabGlyphs();
      verticalSlide();
      grabRelic();
      colorArm();

      // Print values to telemetry for debugging
      telemetry.addData("HS Power", hardware.horizontalSlide.getPower());
      telemetry.addData("Left Pully Power", hardware.leftPully.getPower());
      telemetry.addData("Right Pully Power", hardware.rightPully.getPower());
      telemetry.update();
   }

   private void applyBoost() {
      // Apply boost to drive motors
      if (gamepad1.left_bumper || gamepad1.right_bumper)
         boost = 1;
   }

   private void colorArm() {
      // Color Arm
      if (gamepad2.x) {
         if (hardware.colorServo.getPosition() == hardware.SERVO_MIDDLE)
            hardware.colorServo.setPosition(hardware.SERVO_DOWN);
         else
            hardware.colorServo.setPosition(hardware.SERVO_MIDDLE);
      }
   }

   private void manageDriveTrain() {
      /* DRIVING AND STRAFING */
      if (gamepad1.left_trigger > 0) {
          middlePower = 1; // strafe left
          leftPower = 0;
          rightPower = 0;
      } else if (gamepad1.right_trigger > 0) {
          middlePower = -1; // strafe right
          leftPower = 0;
          rightPower = 0;
      } else {
          // Set motor power to joystick position
          middlePower = 0;
          leftPower = -gamepad1.left_stick_y;
          rightPower = -gamepad1.right_stick_y;
      }

      // Apply drivetrain motor variables to hardware
      hardware.backLeftMotor.setPower(boost * leftPower);
      hardware.backRightMotor.setPower(boost * rightPower);
      hardware.middleMotor.setPower(middlePower);
   }

   private void grabGlyphs() {
      /* MECANUM GLYPH GRABBER */
      if (gamepad2.y)
          mecanumPower = 1; // push glyph out
      else if (gamepad2.a)
          mecanumPower = -1; //pull glyph in
      else
          mecanumPower = 0;

      // Apply mecanum power variables to hardware
      hardware.leftMec.setPower(mecanumPower);
      hardware.rightMec.setPower(mecanumPower);
   }

   private void verticalSlide() {
      /* VERTICAL SLIDE (change grabber vertical position) */
      pullyPower = gamepad2.left_stick_y;
      
      // Apply pully power variables to hardware
      hardware.leftPully.setPower(pullyPower);
      hardware.rightPully.setPower(pullyPower);
   }

   private void grabRelic() {
      // Open/Close relic hand
      if (gamepad2.dpad_left)
          hardware.relicGrabber.setPosition(hardware.RELIC_HAND_CLOSE);
      else if (gamepad2.dpad_right)
          hardware.relicGrabber.setPosition(hardware.RELIC_HAND_OPEN);

      // Move relic wrist
      if (gamepad2.dpad_up)
          hardware.relicWrist.setPosition(hardware.RELIC_OVER);
      else if (gamepad2.dpad_down)
          hardware.relicWrist.setPosition(hardware.RELIC_GRAB);

      // Extend arm
      relicArmPower = gamepad2.right_trigger;

      // Apply horizontal slide power variable to hardware
      hardware.horizontalSlide.setPower(relicArmPower);
   }
}
