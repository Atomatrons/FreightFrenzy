package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.SwitchableLight;

public class Alliance {
  public AllianceType current;
  public AllianceType detectCurrent(ShivaRobot robot){
    // If possible, turn the light on in the beginning (it might already be on anyway,
    // we just make sure it is if we can).
    if (robot.colorSensor instanceof SwitchableLight) {
      ((SwitchableLight)robot.colorSensor).enableLight(true);
    }

    return null;
  }
}
