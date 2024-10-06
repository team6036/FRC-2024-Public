package com.peninsula.frc2024.logging;

import com.peninsula.frc2024.logging.tree.LogTree;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.concurrent.BlockingQueue;

public class ReceiverQueue extends Thread {
  private final BlockingQueue<LogTree> queue;
  private final LogWriter writer;

  public ReceiverQueue(BlockingQueue<LogTree> queue, LogWriter writer) {
    super("ReceiverQueue");
    this.setDaemon(true);
    this.queue = queue;
    this.writer = writer;
  }

  boolean set = false;

  @Override
  public void run() {
    writer.start();

    try {
      while (true) {
        LogTree latest = queue.take();

        if (!set && DriverStation.isFMSAttached()) {
          try {
            writer.log.setFilename(
                DriverStation.getMatchType().toString()
                    + "_"
                    + DriverStation.getMatchNumber()
                    + ".wpilog");
          } catch (Exception ignored) {
          }
          set = true;
        }

        writer.putData(latest);
      }
    } catch (InterruptedException e) {
      writer.end();
    }
  }
}
