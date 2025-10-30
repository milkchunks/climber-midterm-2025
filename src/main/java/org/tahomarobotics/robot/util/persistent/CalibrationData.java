/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
package org.tahomarobotics.robot.util.persistent;

import edu.wpi.first.wpilibj.Filesystem;
import org.tinylog.Logger;

import java.io.*;
import java.util.Arrays;

@SuppressWarnings("unchecked")
public class CalibrationData<T extends Serializable> {
    private static final File HOME_DIR = Filesystem.getOperatingDirectory();

    private final File file;
    private T[] data;

    private boolean calibrated;

    /**
     * Take care of reading and writing of calibration data to a file on the robot.
     *
     * @param filename    - simple filename
     * @param defaultData - data is initialized to these values when no file is found
     */
    public CalibrationData(String filename, T defaultData) {
        this.data = castData(defaultData);
        this.file = new File(HOME_DIR, filename);
        this.calibrated = false;

        if (file.exists()) {
            readCalibrationFile();
        } else {
            Logger.error("Calibration file does not exist: <{}>", file.getAbsolutePath());
        }
    }

    @SuppressWarnings("unchecked")
    private void readCalibrationFile() {
        try (ObjectInputStream inputStream = new ObjectInputStream(new FileInputStream(file))) {
            data = castData((T) inputStream.readObject());
            calibrated = true;
            Logger.info("Successfully read calibration data <{}> -> {}", file.getAbsolutePath(), formatData());
        } catch(Exception e) {
            Logger.error(e, "Failed to read calibration data <{}>", file.getAbsolutePath());
        }
    }

    private void writeCalibrationFile(T[] data) {
        try (ObjectOutputStream outputStream = new ObjectOutputStream(new FileOutputStream(file))) {
            outputStream.writeObject(data[0]);
            Logger.warn("Wrote new calibration data <{}> -> {}", file.getAbsolutePath(), formatData());
        } catch (Exception e) {
            Logger.error(e, "Failed to write calibration data <{}>", file.getAbsolutePath());
        }
    }

    /**
     * Returns the calibration data read from file if successful otherwise the default data
     */
    public T get() {
        return data[0];
    }

    public boolean isCalibrated() {
        return calibrated;
    }

    /**
     * Applies a new set of data to the configuration file
     */
    public void set(T data) {
        this.data = castData(data);
        writeCalibrationFile(this.data);
    }

    private T[] castData(T data) {
        return (T[]) new Serializable[]{data};
    }

    private String formatData() {
        String tmp = Arrays.deepToString(data).substring(1);
        return tmp.substring(0, tmp.length() - 1);
    }

    @Override
    public String toString() {
        return formatData();
    }
}
