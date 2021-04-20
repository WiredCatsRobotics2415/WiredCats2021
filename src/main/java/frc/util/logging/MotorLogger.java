package frc.util.logging;

import java.util.ArrayList;
import java.util.List;

public class MotorLogger implements Runnable {
    private final ArrayList<double[]> data;
    private final Loggable device;
    private long initialTime;
    private boolean started;

    public MotorLogger(Loggable device) {
        this.device = device;
        this.data = new ArrayList<double[]>(20);
        this.initialTime = System.currentTimeMillis();
        this.started = false;
    }

    public void run() {
        if (!this.started) {
            this.initialTime = System.currentTimeMillis();
            this.started = true;
        }
        double[] deviceData = device.getLogOutput();
        double[] entry = new double[deviceData.length + 1];
        entry[0] = (System.currentTimeMillis() - initialTime) / 1000.0;
        for (int i = 1; i < entry.length; i++) {
            entry[i] = deviceData[i - 1];
        }
        data.add(entry);
    }

    public List<double[]> getData() {
        return this.data;
    }

    public void saveDataToCSV(String filename) {
        CSVSaver.saveFile(filename, this.data);
    }
}
