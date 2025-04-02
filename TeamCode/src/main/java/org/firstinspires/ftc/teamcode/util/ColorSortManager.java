package org.firstinspires.ftc.teamcode.util;

public class ColorSortManager {
    private boolean isRedColorSortActive;

    public ColorSortManager() {
        isRedColorSortActive = true; // Default to red
    }

    public boolean isRedColorSortActive() {
        return isRedColorSortActive;
    }

    public void toggleColorSort() {
        isRedColorSortActive = !isRedColorSortActive;
    }
}