/*
 * MIT License
 *
 * Copyright (c) 2025 Bear Metal
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.tahomarobotics.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.List;

/**
 * Utility class for measuring and logging execution times of robot operations.
 * Uses FPGA timestamp for high-precision timing and logs results via AdvantageKit.
 * All times are recorded in milliseconds under the "ExecTimes/" namespace.
 */
public class ExecutionTimeLogger {
    /** Conversion factor from FPGA microseconds to milliseconds for human-readable logging */
    private static final double MICROSECONDS_TO_MILLISECONDS = 0.001;
    
    /** GC beans for potential future GC time compensation (currently unused) */
    //private static final List<GarbageCollectorMXBean> gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
    //private static final long[] lastGCTimes = new long[gcBeans.size()];

    /**
     * Measures and logs the execution time of a void operation.
     * 
     * @param logName The name to use in the log entry (will be prefixed with "ExecTimes/")
     * @param task The operation to time and execute
     * @throws IllegalArgumentException if task is null
     */
    public static void logExecutionTime(String logName, Runnable task) {
        if (task == null) {
            throw new IllegalArgumentException("Task cannot be null");
        }
        
        long startTime = RobotController.getFPGATime();
        try {
            task.run();
        } finally {
            long endTime = RobotController.getFPGATime();
            double executionTimeMs = (endTime - startTime) * MICROSECONDS_TO_MILLISECONDS;
            Logger.recordOutput("ExecTimes/" + logName, executionTimeMs);
        }
    }

    /**
     * Measures and logs the execution time of an operation that returns a value.
     * 
     * @param <T> The return type of the supplier
     * @param logName The name to use in the log entry (will be prefixed with "ExecTimes/")
     * @param supplier The operation to time and execute
     * @return The result from the supplier
     * @throws IllegalArgumentException if supplier is null
     */
    public static <T> T logExecutionTime(String logName, java.util.function.Supplier<T> supplier) {
        if (supplier == null) {
            throw new IllegalArgumentException("Supplier cannot be null");
        }
        
        long startTime = RobotController.getFPGATime();
        try {
            return supplier.get();
        } finally {
            long endTime = RobotController.getFPGATime();
            double executionTimeMs = (endTime - startTime) * MICROSECONDS_TO_MILLISECONDS;
            Logger.recordOutput("ExecTimes/" + logName, executionTimeMs);
        }
    }

}
