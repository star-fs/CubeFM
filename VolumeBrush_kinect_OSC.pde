/**
 * a volume drawing utility with OSC interface for the kinect.
 * Star Morin 2012 2014
 *
 * adapted from:
 * VolumeBrush demo
 * Copyright (c) 2009 Karsten Schmidt
 * http://creativecommons.org/licenses/LGPL/2.1/
 * 
 */

import oscP5.*;
import netP5.*;
OscP5 oscIn;

import toxi.geom.*;
import toxi.geom.mesh.*;
import toxi.volume.*;
import toxi.math.waves.*;
import toxi.processing.*;

import processing.opengl.*;

int DIMX=70;
int DIMY=70;
int DIMZ=70;

float ISO_THRESHOLD = 0.1;
Vec3D SCALE=new Vec3D(1,1,1).scaleSelf(100);

VolumetricSpace volume;
VolumetricBrush brush;
IsoSurface surface;
TriangleMesh mesh;

ToxiclibsSupport gfx;

boolean isWireframe=false;
boolean doSave=false;

float currScale=4;
float density=0.125;

// rotation of x in degrees 
float rot_x = 0;

// rotation of y in degrees 
float rot_y = 0;

// z offset
float z_mod = 47.0;

// kinect angle to initialize to
int tilt_deg = 30;

// the kinect's "x"
int kinectX = 0;

// the kinect's "y"
int kinectY = 0;

// we use depth for a control value - the pen size
float control_z = 0;

float do_draw = 0.0;

float a = 0;

int frm_count = 0;

// address of the ipad
String ipad_addr = "192.168.1.11";

int cur_idx = 0;
int play = 0;
int show_play_point = 0;
int vis_count = 0;

int idx = 0;
Vec3D[] stack;

NetAddress oscOut;
NetAddress oscAudOut;

// bounding cube for visual cue
AABB cube=new AABB(new Vec3D(0,0,0),new Vec3D(50,50,50));

int play_mod = 1000;

void setup() {
  size(1024,768,OPENGL);
  
  // kinect data in
  oscIn  = new OscP5(this,12000);
  oscOut = new NetAddress(ipad_addr,9000);
  oscAudOut = new NetAddress("127.0.0.1",12500);

  // reset the ipad interface
  OscMessage tempo_msg = new OscMessage("/1/tempo");
  tempo_msg.add(0.1375);
  oscIn.send(tempo_msg,oscOut);

  OscMessage audio_msg = new OscMessage("/1/audio_mod");
  audio_msg.add(20.0);
  oscIn.send(audio_msg,oscOut);

  OscMessage env_msg = new OscMessage("/1/note_env");
  env_msg.add(10.0);
  oscIn.send(env_msg,oscOut);

  OscMessage xy_msg = new OscMessage("/1/xy1");
  xy_msg.add(0.0);
  xy_msg.add(0.0);
  oscIn.send(xy_msg,oscOut);

  // reset the audio patch 
  oscIn.send(audio_msg,oscAudOut);
  oscIn.send(env_msg,oscAudOut);
  
  // gl volume stuffs
  hint(ENABLE_OPENGL_4X_SMOOTH);
  gfx=new ToxiclibsSupport(this);
  strokeWeight(0.5);
  volume=new VolumetricSpaceArray(SCALE,DIMX,DIMY,DIMZ);
  brush=new BoxBrush(volume,SCALE.x/2);
  surface=new ArrayIsoSurface(volume);
  mesh=new TriangleMesh();  

  stack = new Vec3D[1000];
  idx = 0;
  cur_idx = 0;
  play = 0;
  show_play_point = 0;
  vis_count = 0;
  
}

void reset() {
  
  // reset the drawing state
  hint(ENABLE_OPENGL_4X_SMOOTH);
  gfx=new ToxiclibsSupport(this);
  strokeWeight(0.5);
  volume=new VolumetricSpaceArray(SCALE,DIMX,DIMY,DIMZ);
  brush=new BoxBrush(volume,SCALE.x/2);
  surface=new ArrayIsoSurface(volume);
  mesh=new TriangleMesh();
  rot_x = 0.0;
  rot_y = 0.0;
  z_mod = 47.0;
  
  // reset the ipad interface
  OscMessage tempo_msg = new OscMessage("/1/tempo");
  tempo_msg.add(0.1375);
  oscIn.send(tempo_msg,oscOut);

  OscMessage audio_msg = new OscMessage("/1/audio_mod");
  audio_msg.add(20.0);
  oscIn.send(audio_msg,oscOut);

  OscMessage env_msg = new OscMessage("/1/note_env");
  env_msg.add(10.0);
  oscIn.send(env_msg,oscOut);

  OscMessage xy_msg = new OscMessage("/1/xy1");
  xy_msg.add(0.0);
  xy_msg.add(0.0);
  oscIn.send(xy_msg,oscOut);

  // reset the audio patch 
  oscIn.send(audio_msg,oscAudOut);
  oscIn.send(env_msg,oscAudOut);
  
  stack = new Vec3D[1000];
  idx = 0;
  cur_idx = 0;
  play = 0;
  show_play_point = 0;
  vis_count = 0;  

}

long time_diff = 0;
long last_time = 0;

void draw() {

  //control_z = 10;
  background(0);
  translate(width/2,height/2,0);
  lightSpecular(230,230,230);
  //directionalLight(255,222,0,1,1,-1);
  directionalLight(255,255,255,1,1,-1);
  
  shininess(1.0);
  rotateX(radians(rot_x*0.5));
  rotateY(radians(rot_y*0.5));
  scale(currScale);
  //beginShape(TRIANGLES);
  beginShape(POINTS);

  // this is the size of the "brush"
  //brush.setSize(control_z);
  brush.setSize(10);

  // this vector defines the position of the brush
  //Vec3D kinectPos=new Vec3D((kinectX-width/2) * 0.5,(kinectY-height/2) * 0.5, z_mod);
  Vec3D kinectPos=new Vec3D((kinectX-width/2) * 0.5,(kinectY-height/2) * 0.5, control_z);

  // handle translation from change of view
  if (rot_x != 0) {
    kinectPos = kinectPos.rotateX(radians(rot_x*0.5));//((rot_x*0.5) - 360) * -1));
  }

  if (rot_y != 0) {
    kinectPos = kinectPos.rotateY(radians(rot_y*0.5));
  }

  // "paint/draw" the sphere at position indicated
  if (do_draw == 1.0) {
    brush.drawAtAbsolutePos(kinectPos,density);
    volume.closeSides();
    surface.reset();
    surface.computeSurfaceMesh(mesh,ISO_THRESHOLD);
    // store the point in an array to play back later
    stack[idx] = kinectPos;
    idx++;
    do_draw = 0;
  }

  time_diff += System.currentTimeMillis() - last_time;
  last_time = System.currentTimeMillis();

  //println("play: " + play + " time_diff: " + time_diff + " stack len: " + stack.length);
  
  if (play == 1 && stack[cur_idx] != null) {
    if (time_diff > play_mod) {
      show_play_point = 1;
    }
  }

  if (show_play_point == 1) {      
     //gfx.mesh(mesh);
     //noFill();
  
     // grey sphere
     stroke(#e6e6e6);  
     fill(#e6e6e6, 80);
     gfx.sphere(new Sphere(stack[cur_idx],7),7);//control_z),30);
     endShape();

     //beginShape(TRIANGLES);
     noFill();
     vis_count++;

      if (vis_count == 1) {
        OscMessage x_msg = new OscMessage("/1/x");
        x_msg.add(stack[cur_idx].x);
        oscIn.send(x_msg,oscAudOut);
  
        OscMessage y_msg = new OscMessage("/1/y");
        y_msg.add(stack[cur_idx].y);
        oscIn.send(y_msg,oscAudOut);
  
        OscMessage z_msg = new OscMessage("/1/z");
        z_msg.add(stack[cur_idx].z);
        oscIn.send(z_msg,oscAudOut);
      }
    
     if (vis_count >= (play_mod / 32)) {
       
       show_play_point = 0;
       time_diff = 0;
       cur_idx++;
       vis_count = 0;
       if (stack[cur_idx] == null) {
         cur_idx = 0;
       }
     }
     // complementary color of fs yellow
     stroke(0,160,255);
     //fill(0,160,255,40);
     gfx.box(cube);
   }
  
  if (mousePressed) {
    brush.drawAtAbsolutePos(kinectPos,density);
    volume.closeSides();
    surface.reset();
    surface.computeSurfaceMesh(mesh,ISO_THRESHOLD);
  }

  // save to disk (or print..) if requested by the user
  if (doSave) {
    // save mesh as STL or OBJ file
    mesh.saveAsSTL(sketchPath("connect_"+(System.currentTimeMillis()/1000)+".stl"));
    doSave=false;
  }

  if (isWireframe) {
    stroke(255);
    noFill();
  } else {
    noStroke();
    fill(255);
  }
  
  gfx.mesh(mesh);
  noFill();

  if (show_play_point == 0) {

    if (play == 0) {
      // grey sphere
      stroke(#e6e6e6);  
      fill(#e6e6e6, 80);
      gfx.sphere(new Sphere(kinectPos,10),10);//control_z),30);
      endShape();
    }
    
    //beginShape(TRIANGLES);
    noFill();
    // complementary color of fs yellow
    stroke(0,160,255);
    //fill(0,160,255,40);
    gfx.box(cube);
  }

  frm_count++;
    
}

/* kinect and ipad touchosc input */
void oscEvent(OscMessage msg) {
    
  // input from the connect ctl patch
  if (msg.checkAddrPattern("/kinect") == true) {
    kinectX = (int)msg.get(0).floatValue();
    kinectY = (int)msg.get(1).floatValue();
    control_z = msg.get(2).floatValue();
  }

  // rotate the scene and trnaslate the pen
  if (msg.checkAddrPattern("/1/xy1") == true) {
    rot_x = msg.get(0).floatValue();  // get the first osc argument
    rot_y = msg.get(1).floatValue(); // get the second osc argument    
  }

  if (msg.checkAddrPattern("/1/draw") == true) {
    do_draw = msg.get(0).floatValue();  // get the first osc argument
  }
  
  // forward ipad events to the audio patch
  if (msg.checkAddrPattern("/1/audio_mod") == true) {    
    OscMessage am_msg = new OscMessage("/1/audio_mod");
    am_msg.add(msg.get(0).floatValue());
    oscIn.send(am_msg,oscAudOut);
  }

  if (msg.checkAddrPattern("/1/note_env") == true) {    
    OscMessage ne_msg = new OscMessage("/1/note_env");
    ne_msg.add(msg.get(0).floatValue());
    oscIn.send(ne_msg,oscAudOut);
  }

  if (msg.checkAddrPattern("/1/scale/1/1") == true) {    
    oscIn.send(msg,oscAudOut);
    
    OscMessage s12a_msg = new OscMessage("/1/scale/1/2");
    s12a_msg.add(0);
    oscIn.send(s12a_msg,oscOut);
    OscMessage s13a_msg = new OscMessage("/1/scale/1/3");
    s13a_msg.add(0);
    oscIn.send(s13a_msg,oscOut);
    OscMessage s14a_msg = new OscMessage("/1/scale/1/4");
    s14a_msg.add(0);
    oscIn.send(s14a_msg,oscOut);

  }

  if (msg.checkAddrPattern("/1/scale/1/2") == true) {    
    oscIn.send(msg,oscAudOut);

    OscMessage s11b_msg = new OscMessage("/1/scale/1/1");
    s11b_msg.add(0);
    oscIn.send(s11b_msg,oscOut);
    OscMessage s13b_msg = new OscMessage("/1/scale/1/3");
    s13b_msg.add(0);
    oscIn.send(s13b_msg,oscOut);
    OscMessage s14b_msg = new OscMessage("/1/scale/1/4");
    s14b_msg.add(0);
    oscIn.send(s14b_msg,oscOut);
  }
  
  if (msg.checkAddrPattern("/1/scale/1/3") == true) {    
    oscIn.send(msg,oscAudOut);

    OscMessage s11c_msg = new OscMessage("/1/scale/1/1");
    s11c_msg.add(0);
    oscIn.send(s11c_msg,oscOut);
    OscMessage s12c_msg = new OscMessage("/1/scale/1/2");
    s12c_msg.add(0);
    oscIn.send(s12c_msg,oscOut);
    OscMessage s14c_msg = new OscMessage("/1/scale/1/4");
    s14c_msg.add(0);
    oscIn.send(s14c_msg,oscOut);
  }
  
  if (msg.checkAddrPattern("/1/scale/1/4") == true) {
    oscIn.send(msg,oscAudOut);

    OscMessage s11d_msg = new OscMessage("/1/scale/1/1");
    s11d_msg.add(0);
    oscIn.send(s11d_msg,oscOut);
    OscMessage s12d_msg = new OscMessage("/1/scale/1/2");
    s12d_msg.add(0);
    oscIn.send(s12d_msg,oscOut);
    OscMessage s13d_msg = new OscMessage("/1/scale/1/3");
    s13d_msg.add(0);
    oscIn.send(s13d_msg,oscOut);
  }

  if ((msg.checkAddrPattern("/1/reset") == true) && (msg.get(0).floatValue() == 1)) {
    reset();
  }

  if ((msg.checkAddrPattern("/1/play") == true) && (msg.get(0).floatValue() == 1)) {
    if (stack[0] != null) {
      play = 1;
    }
  }

  if ((msg.checkAddrPattern("/1/play") == true) && (msg.get(0).floatValue() == 0)) {
    play = 0;
  }

  if ((msg.checkAddrPattern("/1/save") == true) && (msg.get(0).floatValue() == 1)) {
    doSave=true;
  }

  if (msg.checkAddrPattern("/1/tempo") == true) {
    play_mod =  int(msg.get(0).floatValue() * 1000);
  }

}

void keyPressed() {
  if(key=='-') currScale=max(currScale-0.1,0.5);
  if(key=='=') currScale=min(currScale+0.1,10);
  if(key=='w') {  
    isWireframe=!isWireframe; 
    return;
  }
  
  if(key=='s') { 
    doSave=true; 
    return;
  }

  if(key=='p') { 
    play=1; 
    return;
  }

  if(key=='z') { 
    play=0;
    show_play_point = 0; 
    return;
  }

/*
  if(key==' ') { 
    do_draw=1.0; 
    return;
  }
*/
  if (key=='0') density=0.5;
  if (key>='a' && key<='z') ISO_THRESHOLD=(key-'a')*0.019+0.01;
}

