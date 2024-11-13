package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.LogSystem;

import android.os.Environment;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

public class CSVWriter {
    private File file;
    private FileWriter fileWriter;

    public CSVWriter(String folderName, String filename) throws IOException {
        // Create the directory "TAMOPORCA" in the Control Hub's external storage
        File directory = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS), folderName);
        if (!directory.exists()) {
            directory.mkdirs();
        }

        // Create the file BTS_DATA.csv in the TAMOPORCA folder
        file = new File(directory, filename + ".csv");
        fileWriter = new FileWriter(file, true); // 'true' enables appending to the file if it exists
    }

    public void writeRow(List<Object> data) throws IOException {
        StringBuilder row = new StringBuilder();
        for (Object datum : data) {
            row.append(datum.toString()).append(",");
        }
        row.deleteCharAt(row.length() - 1);
        row.append("\n");
        fileWriter.write(row.toString());
        fileWriter.flush();
    }

    public void close() throws IOException {
        fileWriter.close();
    }
}