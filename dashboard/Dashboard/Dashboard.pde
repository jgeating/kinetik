import grafica.*;

GPlot plot;
GPlot plot2;

int startTime = 0;
int lastStepTime = 0;
int totalPoints = 0;


void setup() {
  size(2000, 1000);
  background(150);
  
  // Create a new plot and set its position on the screen
  plot = new GPlot(this);
  plot.setPos(25, 25);
  plot.setDim(800, 300);
  //plot.setTicksLength(-4);
  plot.setHorizontalAxesTicksSeparation(1);
  
  // Set the plot title and the axis labels
  plot.setTitleText("Gyro Angle");
  plot.getXAxis().setAxisLabelText("Time");
  plot.getYAxis().setAxisLabelText("Yaw");
 
  setXAxis(plot);
  plot.setYLim(0, 360); 
  
  
  // Create a new plot and set its position on the screen
  plot2 = new GPlot(this);
  plot2.setPos(25, 425);
  plot2.setDim(800, 300);
  //plot.setTicksLength(-4);
  plot2.setHorizontalAxesTicksSeparation(1);
  
  // Set the plot title and the axis labels
  plot2.setTitleText("Velocity");
  plot2.getXAxis().setAxisLabelText("Time");
  plot2.getYAxis().setAxisLabelText("Velocity (ft/s)");
 
  setXAxis(plot2);
  plot2.setYLim(-10, 10); 
  
  // time
  lastStepTime = millis();
  startTime = millis();
}

public void draw() {
  
  if (millis() - lastStepTime > 100) {
    lastStepTime = millis();
    if (totalPoints >= 100) {
      plot.removePoint(0);
    }
    plot.addPoint((lastStepTime - startTime) / 1000.0, random(360));
    
    plot2.addPoint((lastStepTime - startTime) / 1000.0, random(20) - 10);

    totalPoints++;
  }
  
  setXAxis(plot);
  drawPlot(plot);
  
  setXAxis(plot2);
  drawPlot(plot2);
}

public void drawPlot(GPlot plot) {
  plot.beginDraw();
  plot.drawBackground();
  plot.drawBox();
  plot.drawXAxis();
  plot.drawYAxis();
  plot.drawTitle();
  plot.drawLines();
  plot.drawGridLines(GPlot.HORIZONTAL);
  plot.getMainLayer().drawPoints();
  plot.endDraw(); 
}

public void setXAxis(GPlot plot) {
  float plotStartTime = (lastStepTime - startTime) / 1000.0;
  float xStart = Math.max(plotStartTime - 10, 0);
  float xEnd = xStart + 10;
  plot.setXLim(xStart, xEnd); 
}
