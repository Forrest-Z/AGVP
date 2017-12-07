package com.bmhri.gvp.agvp;

import java.util.UUID;

/**
 * Created by shydh on 5/8/17.
 */

class TaskItem {
    private UUID mId;
    private String mTitle;

    TaskItem() {
        mId = UUID.randomUUID();
    }

    public UUID getId() {
        return mId;
    }

    void setTitle(String mTitle) {
        this.mTitle = mTitle;
    }

    String getTitle() {
        return mTitle;
    }
}
