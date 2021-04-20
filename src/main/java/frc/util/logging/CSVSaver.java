package frc.util.logging;

import java.util.List;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;

public class CSVSaver {
    public static void saveFile(String filename, List<double[]> data) {
        File file = null;
        String path = "/home/lvuser/logging";
        file = new File("/home/lvuser/logging");
        System.out.println("test");
        if (!file.exists()) {
            if (file.mkdir()) {
                System.out.println("Log Directory is created!");
            } else {
                System.out.println("Failed to create Log directory!");
            }
        }
        path += "/" + filename;
        file = new File(path);
        saveFile(file, data);
    }

    public static void saveFile(File file, List<double[]> data) {
        if (file.exists()) {
            System.out.println("deleted files");
            file.delete();
        }
        FileWriter fr = null;
        BufferedWriter br = null;
        String line;
        try {
            fr = new FileWriter(file);
            br = new BufferedWriter(fr);
            for (double[] datum : data) {
                line = "";
                for (int j = 0; j < datum.length; j++) {
                    line += datum[j];
                    if (j < datum.length - 1) {
                        line += ",";
                    }
                }
                line += "\n";
                br.write(line);
            }
        } catch (IOException e) {
            System.out.println(e);
        } finally {
            try {
                br.close();
                fr.close();
            } catch (Exception e) {
                System.out.println(e);
            }
        }
    }
}
