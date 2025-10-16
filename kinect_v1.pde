import SimpleOpenNI.*;
import controlP5.*;

// install https://www.microsoft.com/en-us/download/details.aspx?id=40278

SimpleOpenNI kinect;
ControlP5 cp5;

PGraphics heatmapLayer;
float maxDepth = 1850;
float gradThresh = 75;
float gradExp = 0.21;
boolean showUI = true;
boolean useInterpolation = false;

int originalWidth = 640;
int originalHeight = 480;
int interpFactor = 1; // 1 = no interpolation

int interpWidth = originalWidth * interpFactor;
int interpHeight = originalHeight * interpFactor;

boolean interpFactorChanged = false;
int interpFactorNewValue = interpFactor;


void setup() {
  surface.setResizable(true);
  fullScreen(P3D,2);

  kinect = new SimpleOpenNI(this);
  if (!kinect.isInit()) {
    println("Kinect not found!");
    exit();
  }

  kinect.enableDepth();
  delay(100);

  cp5 = new ControlP5(this);
  createUI();

  heatmapLayer = createGraphics(originalWidth, originalHeight, P2D);
}

void draw() {
  background(0);
  kinect.update();

  if (interpFactorChanged) {
    interpFactor = interpFactorNewValue;
    interpWidth = originalWidth * interpFactor;
    interpHeight = originalHeight * interpFactor;
    interpFactorChanged = false;
  }

  int[] depthMap = kinect.depthMap();
  float[] depthF = new float[depthMap.length];
  for (int i = 0; i < depthMap.length; i++) {
    depthF[i] = constrain(depthMap[i], 0, maxDepth);
  }

  float[] grad;
  int drawWidth, drawHeight;

  if (useInterpolation && interpFactor > 1) {
    interpWidth = originalWidth * interpFactor;
    interpHeight = originalHeight * interpFactor;
    float[] interpDepth = new float[interpWidth * interpHeight];

    // Bilinear interpolation of depthF
    for (int y = 0; y < interpHeight; y++) {
      float gy = ((float) y) / (interpHeight - 1) * (originalHeight - 1);
      int y0 = floor(gy);
      int y1 = min(y0 + 1, originalHeight - 1);
      float wy = gy - y0;

      for (int x = 0; x < interpWidth; x++) {
        float gx = ((float) x) / (interpWidth - 1) * (originalWidth - 1);
        int x0 = floor(gx);
        int x1 = min(x0 + 1, originalWidth - 1);
        float wx = gx - x0;

        float top = depthF[x0 + y0 * originalWidth] * (1 - wx) + depthF[x1 + y0 * originalWidth] * wx;
        float bottom = depthF[x0 + y1 * originalWidth] * (1 - wx) + depthF[x1 + y1 * originalWidth] * wx;
        interpDepth[x + y * interpWidth] = top * (1 - wy) + bottom * wy;
      }
    }

    grad = new float[interpWidth * interpHeight];
    for (int y = 1; y < interpHeight - 1; y++) {
      for (int x = 1; x < interpWidth - 1; x++) {
        int idx = x + y * interpWidth;
        float dx = interpDepth[idx] - interpDepth[idx - 1];
        float dy = interpDepth[idx] - interpDepth[idx - interpWidth];
        grad[idx] = sqrt(dx * dx + dy * dy);
      }
    }

    drawWidth = interpWidth;
    drawHeight = interpHeight;

  } else {
    grad = new float[originalWidth * originalHeight];
    for (int y = 1; y < originalHeight - 1; y++) {
      for (int x = 1; x < originalWidth - 1; x++) {
        int idx = x + y * originalWidth;
        float dx = depthF[idx] - depthF[idx - 1];
        float dy = depthF[idx] - depthF[idx - originalWidth];
        grad[idx] = sqrt(dx * dx + dy * dy);
      }
    }

    drawWidth = originalWidth;
    drawHeight = originalHeight;
  }

  if (heatmapLayer.width != drawWidth || heatmapLayer.height != drawHeight) {
    heatmapLayer = createGraphics(drawWidth, drawHeight, P2D);
  }

  heatmapLayer.beginDraw();
  heatmapLayer.loadPixels();
  for (int i = 0; i < grad.length; i++) {
    float val = grad[i];
    if (val > gradThresh) val = 0;
    val = pow(val / gradThresh, gradExp);
    if (val == 0) {
      heatmapLayer.pixels[i] = color(0);
    } else {
      heatmapLayer.pixels[i] = getColormapHot(val);
    }
  }
  heatmapLayer.updatePixels();
  heatmapLayer.endDraw();

  float scaleFactor = min((float)width / drawWidth, (float)height / drawHeight);
  float offsetX = (width - drawWidth * scaleFactor) / 2;
  float offsetY = (height - drawHeight * scaleFactor) / 2;

  pushMatrix();
  translate(offsetX, offsetY);
  scale(scaleFactor);
  image(heatmapLayer, 0, 0);
  popMatrix();
}

void createUI() {
  cp5.addSlider("maxDepth")
     .setPosition(20, 20)
     .setRange(500, 10000)
     .setValue(maxDepth)
     .setSize(200, 16);

  cp5.addSlider("gradThresh")
     .setPosition(20, 50)
     .setRange(10, 1000)
     .setValue(gradThresh)
     .setSize(200, 16);

  cp5.addSlider("gradExp")
     .setPosition(20, 80)
     .setRange(0.1, 3.0)
     .setValue(gradExp)
     .setSize(200, 16);

  cp5.addToggle("useInterpolation")
     .setPosition(20, 110)
     .setLabel("Use Interpolation")
     .setValue(useInterpolation)
     .setSize(50, 16)
     .onChange(new CallbackListener() {
       public void controlEvent(CallbackEvent event) {
         boolean val = cp5.getController("useInterpolation").getValue() == 1;
         useInterpolation = val;
         cp5.getController("interpFactor").setVisible(val);
       }
     });

  cp5.addSlider("interpFactor")
     .setPosition(20, 140)
     .setRange(1, 8)
     .setValue(interpFactor)
     .setNumberOfTickMarks(8)
     .setSize(200, 16)
     .setVisible(false)
     .onRelease(new CallbackListener() {
       public void controlEvent(CallbackEvent event) {
         interpFactorNewValue = (int)cp5.getController("interpFactor").getValue();
         interpFactorChanged = true;
       }
     });
}

void keyPressed() {
  if (key == 'u' || key == 'U') {
    showUI = !showUI;
    cp5.setVisible(showUI);
  }
}

int getColormapHot(float value) {
  value = constrain(value, 0, 1);
  if (value < 0.33) {
    float v = map(value, 0, 0.33, 0, 255); 
    return color(v, 0, 0); // dark → bright red
  } else if (value < 0.66) {
    float v = map(value, 0.33, 0.66, 0, 255);
    return color(255, v, 0); // red → yellow
  } else {
    float v = map(value, 0.66, 1.0, 0, 255);
    return color(255, 255, v); // yellow → white
  }
}

/*
int getColormapHot(float value) {
  value = constrain(value, 0, 1);
  colorMode(RGB, 255, 255, 255, 255); // ensure RGB

  // Color stops (tweak if you want a different vibe)
  int darkPink     = color(60, 0, 50);      // deep pink/magenta
  int brightPink   = color(255, 70, 182);   // hot pink
  int turquoise    = color(0, 213, 198);    // turquoise blue
  int whiteYellow  = color(255, 247, 176);  // soft white-yellow

  if (value < 1.0/3.0) {
    float t = map(value, 0, 1.0/3.0, 0, 1);
    return lerpColor(darkPink, brightPink, t);
  } else if (value < 2.0/3.0) {
    float t = map(value, 1.0/3.0, 2.0/3.0, 0, 1);
    return lerpColor(brightPink, turquoise, t);
  } else {
    float t = map(value, 2.0/3.0, 1.0, 0, 1);
    return lerpColor(turquoise, whiteYellow, t);
  }
}
*/
