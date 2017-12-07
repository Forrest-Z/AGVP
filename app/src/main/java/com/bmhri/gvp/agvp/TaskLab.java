package com.bmhri.gvp.agvp;

import android.util.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by shydh on 5/8/17.
 */

class TaskLab {

    private static final String TAG = "TaskLab";

    private static TaskLab sTaskLab;

    private List<TaskItem> mTaskItems;

    public static TaskLab get() {
        if (sTaskLab == null)
            sTaskLab = new TaskLab();
        return sTaskLab;
    }

    List<TaskItem> getTaskItems() {
        return mTaskItems;
    }

    private TaskLab() {
        mTaskItems = new ArrayList<>();

        for (int i = 0; i < 3; i++) {
            TaskItem taskItem = new TaskItem();
            taskItem.setTitle("Task #" + i);
            mTaskItems.add(taskItem);
        }
        Log.i(TAG, "TaskLab created!");
    }

    TaskItem getTask() {
        TaskItem taskItem = mTaskItems.get(0);
        mTaskItems.remove(0);
        return taskItem;
    }
}
