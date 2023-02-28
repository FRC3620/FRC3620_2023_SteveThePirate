package frc.robot;

public interface ILevelingDataSource {
  public class LevelingData {
    String levelingState;
    int levelingStateInt;
    double pitch;

    public LevelingData (String levelingState, int levelingStateInt, double pitch) {
      this.levelingState = levelingState;
      this.levelingStateInt = levelingStateInt;
      this.pitch = pitch;
    }
  }

  public LevelingData getLevelingData();

}
