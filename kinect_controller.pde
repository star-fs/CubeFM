/* 
 * kinect-driver control source for Typo C(K)(o)inect Project
 * 
 * Star Morin 2012 2014
 * 
 * using the Processing Wrapper for the OpenNI/Kinect library
 * http://code.google.com/p/simple-openni
 * 
 * derived from pointcloud example:  Max Rheiner / Interaction Design / zhdk / http://iad.zhdk.ch/
 * 
 */

import SimpleOpenNI.*;

SimpleOpenNI context;

import oscP5.*;
import netP5.*;

// net and control data transport
NetAddress theDrawer = new NetAddress("127.0.0.1",12000);
OscP5 oscP5 = new OscP5(this, 9000);
  
float zoomF = 0.3f;

// by default rotate the whole scene 180deg around the x-axis,
float rotX = radians(180);

// the data from openni comes upside down
float rotY = radians(0);

// window of space we wish to examine for control actions
float min_z = 469;
float max_z = 1018;

// number of previous frames to average out position (attempt to reduce jitter)
int avg_frames = 4;

// to speed up the drawing, draw every third point
int steps   = 2;  

float xbuf[] = new float[avg_frames];
float ybuf[] = new float[avg_frames];
float zbuf[] = new float[avg_frames];

int buf_idx = 0;
boolean buf_filled = false;
int frame_count = 0;

void setup()
{

  frameRate(24);
 
  size(1024,768,P3D);  // strange, get drawing error in the cameraFrustum if i use P3D, in opengl there is no problem

  //drawer = new NetAddress("127.0.0.1",12000);

  context = new SimpleOpenNI(this,SimpleOpenNI.RUN_MODE_SINGLE_THREADED);
  //context = new SimpleOpenNI(this);

  // disable mirror
  //context.setMirror(false);

  // enable depthMap generation 
  if(context.enableDepth() == false)
  {
     println("kinect subsys crashed / not connected.  reboot / plug in."); 
     exit();
     return;
  }

  stroke(255,255,255);
  smooth();
  perspective(radians(45),float(width)/float(height),10,150000);

}

void draw()
{
  // update the cam
  context.update();

  background(0,0,0);

  translate(width/2, height/2, 0);
  rotateX(rotX);
  rotateY(rotY);
  scale(zoomF);

  int[]   depthMap = context.depthMap();
  int     index;
  PVector realWorldPoint;
 
  translate(0,0,-1000);  // set the rotation center of the scene 1000 infront of the camera

  stroke(255);

  PVector[] realWorldMap = context.depthMapRealWorld();

  float lx = 0;
  float ly = 0;
  float lz = 3000;  

  for(int y=0;y < context.depthHeight();y+=steps)
  {
    for(int x=0;x < context.depthWidth();x+=steps)
    {
      index = x + y * context.depthWidth();
      
      if(depthMap[index] > 0) 
      {
        realWorldPoint = realWorldMap[index];
        // limit to min - max window
        if (realWorldPoint.z > min_z && realWorldPoint.z < max_z) {

          // is this the closest z we've seen?          
          if (realWorldPoint.z < lz) {
            
            // range scale
            // f(x) = (((b-a)(x - min))/(max - min))+a 
            // invert (x - range) * -1
            
            lx = (((((600 - 130) * (realWorldPoint.x - -350)) / (350 - -350) + 130) - (600 - 130)) * -1) + 450;
            ly = ((((800 - 250) * (realWorldPoint.y - -450)) / (500 - -450) + 250) - (800 - 250 + 300)) * -1;
            lz = ((((72 - 0) * (realWorldPoint.z - min_z)) / (max_z - min_z) + 0) - 72) * -1;
          }
          // draw the projected point          
          //point(realWorldPoint.x,realWorldPoint.y,realWorldPoint.z);
        }
      }
    }
  }

  if (frame_count == avg_frames) {
    buf_filled = true;
  }

  xbuf[buf_idx] = lx;
  ybuf[buf_idx] = ly;
  zbuf[buf_idx] = lz;

  // attempt to average out the points
  // to reduce "jitter" 
  if (buf_filled) {
    
    if (buf_idx == (avg_frames - 1)) {
      buf_idx = 0;
    } else {
      buf_idx++;
    }

    int sum_x = 0;
    float sum_y = 0;
    int sum_z = 0;
      
    for (int i=0;i<=(avg_frames - 1);i++) {
      sum_x += xbuf[i];
      sum_y += ybuf[i];
      sum_z += zbuf[i];
    }
    
    if (sum_x > 0) {
      lx = sum_x / avg_frames;
      ly = sum_y / avg_frames;
      lz = sum_z / avg_frames;
    }

    if (lz > 0 && lz < 750) {
      sendData(lx, ly, lz);
    }

  } else {
    // send the unaveraged points
    if (lz > 0 && lz < 750) {
      sendData(lx, ly, lz);
    }
  }    
  // draw the kinect cam
  context.drawCamFrustum();
  frame_count++;
}

void sendData(float x, float y, float z) {
  OscMessage msg = new OscMessage("/kinect");
  
  z = z - abs(max_z - min_z) * -1;
  
  msg.add(x);
  msg.add(y);
  msg.add((z * -1) + 600);
  
  oscP5.send(msg, theDrawer);
}

void keyPressed()
{
  switch(key)
  {
  case ' ':
    context.setMirror(!context.mirror());
    break;
  case '7':
    min_z++;
    println("min_z up: " + min_z);
    break;
  case '1':
    min_z--;
    println("min_z down: " + min_z);
    break;
  case '9':
    max_z++;
    println("max_z up: " + max_z);
    break;
  case '3':
    max_z--;
    println("max_z down: " + max_z);
    break;
  }

  switch(keyCode)
  {
  case LEFT:
    rotY += 0.1f;
    break;
  case RIGHT:
    // zoom out
    rotY -= 0.1f;
    break;
  case UP:
    if(keyEvent.isShiftDown())
      zoomF += 0.02f;
    else
      rotX += 0.1f;
    break;
  case DOWN:
    if(keyEvent.isShiftDown())
    {
      zoomF -= 0.02f;
      if(zoomF < 0.01)
        zoomF = 0.01;
    }
    else
      rotX -= 0.1f;
    break;
  }
}

