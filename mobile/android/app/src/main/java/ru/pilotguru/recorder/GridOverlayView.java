// Draws a grid on top of the underlying view (which is the video capture preview) for easier
// alignment of the camera.

package ru.pilotguru.recorder;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.View;

public class GridOverlayView extends View {
  static final int GRID_SIZE = 10;
  private final Paint paint = new Paint();

  public GridOverlayView(Context context, AttributeSet attrs) {
    super(context, attrs);
    paint.setColor(Color.LTGRAY);
    paint.setStrokeWidth(1f);
  }

  @Override
  public void onDraw (Canvas canvas) {
    final int height = canvas.getHeight();
    final int height_step = height / GRID_SIZE;
    final int width = canvas.getWidth();
    final int width_step = width / GRID_SIZE;
    for (int i=1; i<GRID_SIZE; ++i) {
      final int x = width_step * i;
      final int y = height_step * i;
      canvas.drawLine(0, y, width, y, paint);
      canvas.drawLine(x, 0, x, height, paint);
    }
  }
}
