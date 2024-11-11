package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Manager;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TAG;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Autonomous
public class Manager extends LinearOpMode {

    private String[][] _tasks;
    private ArrayList<Tasks> _checkedTasks = new ArrayList<>();

    private int pointer = 0;

    private boolean blueSide = false; // if it is blue side autonomous DEFAULT IS RED SIDE
    private boolean nearBasket = false; // if it is near the basket on the start DEFAULT IS AWAY

    private void loadTasks() {
        _tasks = new String[Tasks.values().length][2];


        for(int i = 0; i < Tasks.values().length; i++) {
            _tasks[i][0] = Tasks.values()[i].TaskName;
            _tasks[i][1] = "uncheck";
            telemetry.addLine(_tasks[i][0] + "loaded with value " + _tasks[i][1]);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        loadTasks();

        while(!isStarted()) {

            for(int i = 0; i < _tasks.length; i++) {
                telemetry.addData(_tasks[i][0], pointer == i ? "->" + _tasks[i][1] : _tasks[i][1]);
            }

            if(gamepad1.dpad_up || gamepad2.dpad_up) {
                if(pointer > 0) {
                    pointer -= 1;
                }
            }

            if(gamepad1.dpad_down || gamepad2.dpad_down) {
                if(pointer < (_tasks.length - 1)) {
                    pointer += 1;
                }
            }

            if(gamepad1.x || gamepad2.x) {
                _tasks[pointer][1] = _tasks[pointer][1].equals("uncheck") ? "check" : "uncheck";
                if(_tasks[pointer][1].equals("uncheck")) {
                    _checkedTasks.remove(Tasks.getTask(_tasks[pointer][0]));
                } else {
                    _checkedTasks.add(Tasks.getTask(_tasks[pointer][0]));
                }
            }

            if(gamepad1.a || gamepad2.a) {
                blueSide = !blueSide;
            }
            if(gamepad1.y || gamepad2.y) {
                nearBasket = !nearBasket;
            }

            sleep(10);
            telemetry.addLine();
            telemetry.addLine();

            telemetry.addData("ALLIANCE", blueSide ? "BLUE" : "RED");
            telemetry.addData("SIDE", nearBasket ? "NEAR BASKET" : "NEAR OBS ZONE");
            telemetry.update();

            sleep(150);

            for(Tasks task : _checkedTasks) {
                Log.d("TASK: ", task.TaskName);
                Log.d(TAG, "start: " + task.TaskName);

                if(task.TaskName.equalsIgnoreCase("D_BASKET")) {
                    Log.d(this.getClass().getName(), "running: basket");
                    //delivery sample in the high basket
                }
                if(task.TaskName.equalsIgnoreCase("D_OBS_ZONE")) {
                    Log.d(this.getClass().getName(), "running: drop");
                    //drop sample in obs zone
                }
                if(task.TaskName.equalsIgnoreCase("CATCH_SAMPLE")) {
                    Log.d(this.getClass().getName(), "running: catch");
                    //catch sample from submersible
                }
                if(task.TaskName.equalsIgnoreCase("CLIP_SPECIMEN")) {
                    Log.d(this.getClass().getName(), "running: clip");
                    //clip specimen in the high chamber
                }
                if(task.TaskName.equalsIgnoreCase("PARK")) {
                    Log.d(this.getClass().getName(), "running: park");
                    //park on net zone
                }

                telemetry.update();
            }
            for(int i = 0; i < _checkedTasks.size() - 1; i++) {
                Log.d(TAG, "runOpMode: Removed: " + _checkedTasks.get(i).TaskName);
                _checkedTasks.remove(i);
            }
        }

        waitForStart();
        while (opModeIsActive()) {

        }
    }
}