package frc.robot;

public interface ILevelingDataSource {
  public class LevelingData {
    String levelingState;
    double pitch;

    public LevelingData (String levelingState, double pitch) {
      this.levelingState = levelingState;
      this.pitch = pitch;
    }

    public String getLevelingState() {
      return levelingState;
    }

    public double getPitch() {
      return pitch;
    }
  }

  public LevelingData getLevelingData();

}
