package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Manager;

public enum Tasks {
    CATCH_SAMPLE("CATCH_SAMPLE"),
    DELIVERY_BASKET("D_BASKET"),
    DELIVERY_OBS_ZONE("D_OBS_ZONE"),
    CLIP_SPECIMEN("CLIP_SPECIMEN"),
    CATCH_SPECIMEN("CATCH_SPECIMEN"),
    SPIKE_MARK_1("SPIKE_MARK_1"),
    SPIKE_MARK_2("SPIKE_MARK_2"),
    SPIKE_MARK_3("SPIKE_MARK_3"),
    PARK("PARK");
    public final String TaskName;

    public static Tasks getTask(String TaskName) {
        Tasks match = null;

        for (Tasks task : Tasks.values()) {
            if (task.TaskName.equals(TaskName)) {
                match = task;
            }
        }
        return match;
    }

    private Tasks(String TaskName) {
        this.TaskName = TaskName;
    }
}
