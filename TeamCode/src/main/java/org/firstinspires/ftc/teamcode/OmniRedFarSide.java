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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="OmniRedFarSide", group="Linear Opmode")
public class OmniRedFarSide extends LinearOpMode {
   OmniHardware hardware;
   Auto auto;

   @Override
   public void runOpMode() throws InterruptedException {
      hardware = new OmniHardware(hardwareMap);
      hardware.initHardware();
      auto = new Auto(hardware);

      hardware.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      hardware.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      hardware.middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      waitForStart();

      auto.raisePully();

      telemetry.addLine("Calibrating Gyro sensor");
      telemetry.update();
      auto.calibrateGyro();

      telemetry.addLine("Knocking Jewel");
      telemetry.update();
      auto.knockJewel("red");

      telemetry.addLine("Reading Pictograph");
      telemetry.update();
      auto.readPictograph(hardwareMap);

      telemetry.addLine("Correcting Angle");
      telemetry.update();
      auto.correctAngle();

      telemetry.addLine("Driving Forward");
      telemetry.update();
      auto.driveDistance(1, 0.7, auto.encoderTicksForDist(34, hardware.OMNI_CIRCUMFERENCE, hardware.NEVEREST_TICKS_PER_REV));

      telemetry.addLine("Strafing Left");
      telemetry.update();
      auto.cryptoboxInsertion(auto.KEY, "red");
      //auto.strafeDistance(1, 0.5, auto.encoderTicksForDist(dist[1], hardware.OMNI_CIRCUMFERENCE, hardware.NEVEREST_TICKS_PER_REV));

      auto.driveForward(0.1, 1000);
      telemetry.addLine("Placing Glyph");
      telemetry.update();
      auto.mecanumsOut(1000);

      telemetry.addLine("Pushing Glyph to Ensure Placement");
      telemetry.update();
      auto.driveBackward(0.2, 1000);
      auto.driveForward(0.5, 1000);
      auto.driveDistance(-1, 0.3, auto.encoderTicksForDist(-10, hardware.OMNI_CIRCUMFERENCE, hardware.NEVEREST_TICKS_PER_REV));
      auto.lowerPully();
   }
}
