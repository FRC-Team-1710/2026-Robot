package frc.robot;

import frc.robot.constants.GoodExampleConstants;

public class GoodExample {
  public boolean g_active;

  private int m_indexCount;

  GoodExample(int pIndexCount) {
    this.m_indexCount = pIndexCount;

    this.g_active = false;
  }

  public boolean IsIndexFull() {
    if (!this.g_active) return false;

    if (this.m_indexCount >= GoodExampleConstants.MAX_BALL_COUNT) {
      return true;
    }

    return false;
  }
}
