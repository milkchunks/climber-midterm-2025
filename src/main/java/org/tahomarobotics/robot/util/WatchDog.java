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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.tinylog.Logger;

public class WatchDog {

    public static void disableWatchdog(IterativeRobotBase robot) {
        disableWatchdog(robot, IterativeRobotBase.class);
    }
    public static void disableWatchdog(CommandScheduler scheduler) {
        disableWatchdog(scheduler, CommandScheduler.class);
    }
    private static void disableWatchdog(Object inst, Class clazz) {

        try {
            var field = clazz.getDeclaredField("m_watchdog");
            field.setAccessible(true);
            field.set(inst, new NullWatchDog(0, () -> {}));
            Logger.warn("Disabled " + inst.getClass() + "'s watchdog!");
        } catch (NoSuchFieldException | IllegalAccessException e) {
            e.printStackTrace();
        }
    }

    private static void printLoopOverrunMessage() {
        DriverStation.reportWarning("Loop time overrun\n", false);
    }

    private static class NullWatchDog extends edu.wpi.first.wpilibj.Watchdog {

        public NullWatchDog(double timeoutSeconds, Runnable callback) {
            super(timeoutSeconds, callback);
        }

        @Override
        public void addEpoch(String epochName) {}

        @Override
        public void printEpochs() {}

        @Override
        public void enable() {}

        @Override
        public void disable() {}
    }
}