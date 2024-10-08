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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import javax.lang.model.element.VariableElement;

@Autonomous(name="Auto Test 2", group="Robot")

public class AutoTest2 extends LinearOpMode {


    public void runOpMode() {

        RobotHardware robot = new RobotHardware(this);

        robot.Initialize();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Strafe left 27.94 cm
        robot.StrafeLeftCM(0.3, 27.94);

        // add camera detection true/false. if false continue on, if true:
        //drive forword 45.085 cm
        robot.DriveCM(0.3, 45.085);
        //place pixel
        // Strafe left 30.48 cm
        robot.StrafeLeftCM(0.3, 30.48);
        // drive forward 106.68 cm
        robot.DriveCM(0.3, 106.68);
        // strafe right 304.8 cm
        robot.StrafeRightCM(0.3, 304.8);
        // drive backward 30.48 cm
        robot.DriveCM(-0.3, -30.48);

        // strafe right 27.94 cm
        robot.StrafeRightCM(0.3, 27.94);

        // add camera detection true/false. if false continue on, if true:
        //drive forword 45.085 cm
        robot.DriveCM(0.3, 45.085);
        //place pixel
        // Strafe left 60.96 cm
        robot.StrafeLeftCM(0.3, 60.96);
        // drive forward 53.34 cm
        robot.DriveCM(0.3, 53.34);
        // strafe right 304.8 cm
        robot.StrafeRightCM(0.3, 304.8);
        // drive backward 30.48 cm
        robot.DriveCM(-0.3, -30.48);

        // strafe right 27.94 cm
        robot.StrafeRightCM(0.3, 27.94);
        //drive forword 75.565 cm
        robot.DriveCM(0.3, 75.565);
        //place pixel
        // Strafe left 90 cm
        //robot.StrafeLeftCM(0.3, 90);
        // drive forward 106.68 cm
        //robot.DriveCM(0.3, 106.68);
        // strafe right 304.8 cm
        //robot.StrafeRightCM(0.3, 304.8);
        // drive backward 30.48 cm
        //robot.DriveCM(-0.3, -30.48);

        // stop
    }


}
