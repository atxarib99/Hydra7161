package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.util.Util;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Created by Arib on 12/1/2015.
 */
public class FileSaver {
    private String name;
    private String directory;
    private String generated;
    public FileSaver(String name, String directory, String generated) {
        this.name = name;
        this.directory = directory;
        this.generated = generated;
    }
    public void saveAs() {
        try {
            File file = SavingUtil.createFileOnDevice(directory, name + ".txt", false);
            String put = generated;
            FileWriter fw = new FileWriter(file.getAbsoluteFile());
            BufferedWriter bw = new BufferedWriter(fw);
            bw.write(put);
            bw.close();

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
