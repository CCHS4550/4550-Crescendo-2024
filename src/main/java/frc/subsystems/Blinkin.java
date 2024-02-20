package frc.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase {
    private Spark blinkin;
    private HashMap<String, Double> colors;
    private List<String> cols;
    private int ind = 0;
    public Blinkin() {
        blinkin = new Spark(0);
        colors = new HashMap<>();
        colors.put("Rainbow, Rainbow Palette", -0.99);
        colors.put("Rainbow, Party Palette", -0.97);
        colors.put("Rainbow, Ocean Palette", -0.95);
        colors.put("Rainbow, Lave Palette", -0.93);
        colors.put("Rainbow, Forest Palette", -0.91);
        colors.put("Rainbow with Glitter", -0.89);
        colors.put("Confetti", -0.87);
        colors.put("Shot, Red", -0.85);
        colors.put("Shot, Blue", -0.83);
        colors.put("Shot, White", -0.81);
        colors.put("Sinelon, Rainbow Palette", -0.79);
        colors.put("Sinelon, Party Palette", -0.77);
        colors.put("Sinelon, Ocean Palette", -0.75);
        colors.put("Sinelon, Lava Palette", -0.73);
        colors.put("Sinelon, Forest Palette", -0.71);
        colors.put("Beats per Minute, Rainbow Palette", -0.69);
        colors.put("Beats per Minute, Party Palette", -0.67);
        colors.put("Beats per Minute, Ocean Palette", -0.65);
        colors.put("Beats per Minute, Lava Palette", -0.63);
        colors.put("Beats per Minute, Forest Palette", -0.61);
        colors.put("Fire, Medium", -0.59);
        colors.put("Fire, Large", -0.57);
        colors.put("Twinkles, Rainbow Palette", -0.55);
        colors.put("Twinkles, Party Palette", -0.53);
        colors.put("Twinkles, Ocean Palette", -0.51);
        colors.put("Twinkles, Lava Palette", -0.49);
        colors.put("Twinkles, Forest Palette", -0.47);
        colors.put("Color Waves, Rainbow Palette", -0.45);
        colors.put("Color Waves, Party Palette", -0.43);
        colors.put("Color Waves, Ocean Palette", -0.41);
        colors.put("Color Waves, Lava Palette", -0.39);
        colors.put("Color Waves, Forest Palette", -0.37);
        colors.put("Larson Scanner, Red", -0.35);
        colors.put("Larson Scanner, Gray", -0.33);
        colors.put("Light Chase, Red", -0.31);
        colors.put("Light Chase, Blue", -0.29);
        colors.put("Light Chase, Gray", -0.27);
        colors.put("Heartbeat, Red", -0.25);
        colors.put("Heartbeat, Blue", -0.23);
        colors.put("Heartbeat, White", -0.21);
        colors.put("Heartbeat, Gray", -0.19);
        colors.put("Breath, Red", -0.17);
        colors.put("Breath, Blue", -0.15);
        colors.put("Breath, Gray", -0.13);
        colors.put("Strobe, Red", -0.11);
        colors.put("Strobe, Blue", -0.09);
        colors.put("Strobe, Gold", -0.07);
        colors.put("Strobe, White", -0.05);
        colors.put("End to End Blend to Black", -0.03);
        colors.put("Larson Scanner", -0.01);
        colors.put("Color 1 Light Chase", 0.01);
        colors.put("Color 1 Heartbeat Slow", 0.03);
        colors.put("Color 1 Heartbeat Medium", 0.05);
        colors.put("Color 1 Heartbeat Fast", 0.07);
        colors.put("Color 1 Breath Slow", 0.09);
        colors.put("Color 1 Breath Fast", 0.11);
        colors.put("Color 1 Shot", 0.13);
        colors.put("Color 1 Strobe", 0.15);
        colors.put("Color 2 End to End Blend to Black", 0.17);
        colors.put("Color 2 Larson Scanner", 0.19);
        colors.put("Color 2 Light Chase", 0.21);
        colors.put("Color 2 Heartbeat Slow", 0.23);
        colors.put("Color 2 Heartbeat Medium", 0.25);
        colors.put("Color 2 Heartbeat Fast", 0.27);
        colors.put("Color 2 Breath Slow", 0.29);
        colors.put("Color 2 Breath Fast", 0.31);
        colors.put("Color 2 Shot", 0.33);
        colors.put("Color 2 Strobe", 0.35);
        colors.put("Sparkle, Color 1 on Color 2", 0.37);
        colors.put("Sparkle, Color 2 on Color 1", 0.39);
        colors.put("Color Gradient, Color 1 and 2", 0.41);
        colors.put("Beats per Minute, Color 1 and 2", 0.43);
        colors.put("End to End Blend, Color 1 to 2", 0.45);
        colors.put("End to End Blend", 0.47);
        colors.put("Color 1 and Color 2 no blending (Setup Pattern)", 0.49);
        colors.put("Twinkles, Color 1 and 2", 0.51);
        colors.put("Color Waves, Color 1 and 2", 0.53);
        colors.put("Sinelon, Color 1 and 2", 0.55);
        colors.put("Hot Pink", 0.57);
        colors.put("Dark red", 0.59);
        colors.put("Red", 0.61);
        colors.put("Red Orange", 0.63);
        colors.put("Orange", 0.65);
        colors.put("Gold", 0.67);
        colors.put("Yellow", 0.69);
        colors.put("Lawn Green", 0.71);
        colors.put("Lime", 0.73);
        colors.put("Dark Green", 0.75);
        colors.put("Green", 0.77);
        colors.put("Blue Green", 0.79);
        colors.put("Aqua", 0.81);
        colors.put("Sky Blue", 0.83);
        colors.put("Dark Blue", 0.85);
        colors.put("Blue", 0.87);
        colors.put("Blue Violet", 0.89);
        colors.put("Violet", 0.91);
        colors.put("White", 0.93);
        colors.put("Gray", 0.95);
        colors.put("Dark Gray", 0.97);
        colors.put("Black", 0.99);
        cols = new ArrayList<>(colors.keySet());
        ind = 0;
    }

    public void setPattern(String pattern) {
        blinkin.set(colors.get(pattern));
        ind = cols.indexOf(pattern);
    }
    public void toggleColor() {
        ind  = (ind + 1) % cols.size();
        blinkin.set(colors.get(cols.get(ind)));
    }
    public void printColor() {
        System.out.println(cols.get(ind));
    }
}