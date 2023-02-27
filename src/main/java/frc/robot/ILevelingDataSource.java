package frc.robot;

public interface ILevelingDataSource {
  enum LevelingState {
    LEVEL, TILTED, COUNTER, DONE
  }

  public class LevelingData {
    public LevelingState levelingState;
    public double pitch;
  }

  public LevelingData getLevelingData();

}
