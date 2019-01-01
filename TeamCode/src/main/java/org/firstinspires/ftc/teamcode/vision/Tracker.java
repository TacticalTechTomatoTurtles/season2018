package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Point;

import java.util.List;

public class Tracker {

    private Result trackedItem;

    public void setTrackedItem(Result trackedItem) {
        this.trackedItem = trackedItem;
    }

    public Result getTrackedItem() {
        return trackedItem;
    }

    public void update(List<Result> items) {

        if(trackedItem == null) {
            initialize(items);
        } else {
            Point tpos = trackedItem.getCenter();
            double tarea = trackedItem.getArea();

            Result best = null;
            double bestScore = Double.MAX_VALUE;
            for (Result item : items) {
                double area = item.getArea();

                Point pos = item.getCenter();
                double distSq = Math.pow(pos.x - tpos.x, 2) + Math.pow(pos.y - tpos.y, 2);

                double score = distSq + Math.abs(tarea - area);
                if (score < bestScore) {
                    best = item;
                    bestScore = score;
                }
            }

            if (best != null) {
                trackedItem = best;
            }
        }
    }

    public void initialize(List<Result> items) {

        Result best = null;
        double bestScore = Double.MAX_VALUE;
        for(Result item : items) {
            double score = item.getArea() - Math.pow(200 / item.getFrameSize().getHeight() * item.getCenter().y, 2);
            if(score < bestScore) {
                best = item;
                bestScore = score;
            }
        }

        trackedItem = best;
    }
}