package frc.robot;

public interface ILevelingDataSource {
  public class LevelingData {
    String levelingState;
    int levelingStateInt;
    double pitch, power;

    public LevelingData (String levelingState, int levelingStateInt, double pitch, double power) {
      this.levelingState = levelingState;
      this.levelingStateInt = levelingStateInt;
      this.pitch = pitch;
      this.power = power;
    }
  }

  public LevelingData getLevelingData();

}
