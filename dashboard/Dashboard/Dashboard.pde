import grafica.*;

GPlot swerveAnglePlot;
GPlot xVelocityPlot;
GPlot yVelocityPlot;
GPlot vestAnglePlot;
GPlot motorVelocityPlot;
GPlot motorYawPlot;

int startTime = 0;
int lastStepTime = 0;
int totalPoints = 0;


void setup() {
  size(1800, 1200);
  background(150);
  
  // Create a new plot and set its position on the screen
  swerveAnglePlot = new GPlot(this);
  swerveAnglePlot.setPos(0, 0);
  swerveAnglePlot.setDim(800, 300);
  swerveAnglePlot.setHorizontalAxesTicksSeparation(1);
  swerveAnglePlot.setTitleText("Swerve Angle");
  swerveAnglePlot.getXAxis().setAxisLabelText("Time (s)");
  swerveAnglePlot.getYAxis().setAxisLabelText("Angle (deg)");
  setXAxis(swerveAnglePlot);
  swerveAnglePlot.setYLim(0, 360); 
  
  
  // Create a new plot and set its position on the screen
  xVelocityPlot = new GPlot(this);
  xVelocityPlot.setPos(0, 400);
  xVelocityPlot.setDim(800, 300);
  xVelocityPlot.setHorizontalAxesTicksSeparation(1);
  xVelocityPlot.setTitleText("X Velocity");
  xVelocityPlot.getXAxis().setAxisLabelText("Time (s)");
  xVelocityPlot.getYAxis().setAxisLabelText("Velocity (ft/s)");
  setXAxis(xVelocityPlot);
  xVelocityPlot.setYLim(-10, 10); 
  
  // Create a new plot and set its position on the screen
  yVelocityPlot = new GPlot(this);
  yVelocityPlot.setPos(0, 800);
  yVelocityPlot.setDim(800, 300);
  yVelocityPlot.setHorizontalAxesTicksSeparation(1);
  yVelocityPlot.setTitleText("Y Velocity");
  yVelocityPlot.getXAxis().setAxisLabelText("Time (s)");
  yVelocityPlot.getYAxis().setAxisLabelText("Velocity (ft/s)");
  setXAxis(yVelocityPlot);
  yVelocityPlot.setYLim(-10, 10); 
  
  // Create a new plot and set its position on the screen
  vestAnglePlot = new GPlot(this);
  vestAnglePlot.setPos(900, 0);
  vestAnglePlot.setDim(800, 300);
  vestAnglePlot.setHorizontalAxesTicksSeparation(1);
  vestAnglePlot.setTitleText("Vest Angle");
  vestAnglePlot.getXAxis().setAxisLabelText("Time (s)");
  vestAnglePlot.getYAxis().setAxisLabelText("Angle (deg)");
  setXAxis(vestAnglePlot);
  vestAnglePlot.setYLim(0, 360); 
  
  // Create a new plot and set its position on the screen
  motorVelocityPlot = new GPlot(this);
  motorVelocityPlot.setPos(900, 400);
  motorVelocityPlot.setDim(800, 300);
  motorVelocityPlot.setHorizontalAxesTicksSeparation(1);
  motorVelocityPlot.setTitleText("Motor Velocities");
  motorVelocityPlot.getXAxis().setAxisLabelText("Time (s)");
  motorVelocityPlot.getYAxis().setAxisLabelText("Velocity (ft/s)");
  setXAxis(motorVelocityPlot);
  motorVelocityPlot.setYLim(-10, 10); 
  
   // Create a new plot and set its position on the screen
  motorYawPlot = new GPlot(this);
  motorYawPlot.setPos(900, 800);
  motorYawPlot.setDim(800, 300);
  motorYawPlot.setHorizontalAxesTicksSeparation(1);
  motorYawPlot.setTitleText("Motor Yaws");
  motorYawPlot.getXAxis().setAxisLabelText("Time (s)");
  motorYawPlot.getYAxis().setAxisLabelText("Angle (deg)");
  setXAxis(motorYawPlot);
  motorYawPlot.setYLim(0, 360); 
  
  // time
  lastStepTime = millis();
  startTime = millis();
}

public void draw() {
  
  if (millis() - lastStepTime > 100) {
    lastStepTime = millis();
    if (totalPoints >= 100) {
      swerveAnglePlot.removePoint(0);
    }
    swerveAnglePlot.addPoint((lastStepTime - startTime) / 1000.0, random(360));
    xVelocityPlot.addPoint((lastStepTime - startTime) / 1000.0, random(20) - 10);    
    yVelocityPlot.addPoint((lastStepTime - startTime) / 1000.0, random(20) - 10);
    vestAnglePlot.addPoint((lastStepTime - startTime) / 1000.0, random(360));
    motorVelocityPlot.addPoint((lastStepTime - startTime) / 1000.0, random(20) - 10);
    motorYawPlot.addPoint((lastStepTime - startTime) / 1000.0, random(360));

    totalPoints++;
  }
  
  setXAxis(swerveAnglePlot);
  drawPlot(swerveAnglePlot);
  
  setXAxis(xVelocityPlot);
  drawPlot(xVelocityPlot);
  
  setXAxis(yVelocityPlot);
  drawPlot(yVelocityPlot);
  
  setXAxis(vestAnglePlot);
  drawPlot(vestAnglePlot);
  
  setXAxis(motorVelocityPlot);
  drawPlot(motorVelocityPlot);
  
  setXAxis(motorYawPlot);
  drawPlot(motorYawPlot);
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
