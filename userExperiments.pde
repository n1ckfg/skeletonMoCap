 import oscP5.*;
import netP5.*;

/* --------------------------------------------------------------------------
 * SimpleOpenNI User Test
 * --------------------------------------------------------------------------
 * Processing Wrapper for the OpenNI/Kinect 2 library
 * http://code.google.com/p/simple-openni
 * --------------------------------------------------------------------------
 * prog:  Max Rheiner / Interaction Design / Zhdk / http://iad.zhdk.ch/
 * date:  12/12/2012 (m/d/y)
 * ----------------------------------------------------------------------------
 */

import SimpleOpenNI.*;

SimpleOpenNI  context;
color[]       userClr = new color[] { 
  color(255, 0, 0), 
  color(0, 255, 0), 
  color(0, 0, 255), 
  color(255, 255, 0), 
  color(255, 0, 255), 
  color(0, 255, 255)
};

int port=9000;
OscP5 oscP5;
OscMessage message;
NetAddress myRemoteLocation;
PVector com = new PVector();                                   
PVector com2d = new PVector();        
int numJoints=15;
int jointIndex=0;
//head
//neck
//leftShoulder
//leftElbow
//leftHand
//rightShoulder
//rightElbow
//rightHand
//torso
//left hip
//left knee
//left foot
//right hip
//right knee
//right foot

float[] joints=new float[numJoints*9+4];
void setup()
{
  size(640, 480);

  context = new SimpleOpenNI(this);
  if (context.isInit() == false)
  {
    println("Can't init SimpleOpenNI, maybe the camera is not connected!"); 
    exit();
    return;
  }

  // enable depthMap generation 
  context.enableDepth();

  // enable skeleton generation for all joints
  context.enableUser();

  background(200, 0, 0);

  stroke(0, 0, 255);
  strokeWeight(3);
  smooth();
  oscP5 = new OscP5(this,port);
  
  /* myRemoteLocation is a NetAddress. a NetAddress takes 2 parameters,
   * an ip address and a port number. myRemoteLocation is used as parameter in
   * oscP5.send() when sending osc packets to another computer, device, 
   * application. usage see below. for testing purposes the listening port
   * and the port of the remote location address are the same, hence you will
   * send messages back to this sketch.
   */
  myRemoteLocation = new NetAddress("127.0.0.1",port);
}

void draw()
{
  // update the cam
  context.update();

  // draw depthImageMap
  //image(context.depthImage(),0,0);
  image(context.userImage(), 0, 0);

  // draw the skeleton if it's available
  int[] userList = context.getUsers();
  for (int i=0; i<userList.length; i++)
  {
    int user=0;
    if (context.isTrackingSkeleton(userList[i]))
    {
      stroke(userClr[ (userList[i] - 1) % userClr.length ] );
      drawSkeleton(userList[i]);
      //  float confidence = context.getJointOrientationSkeleton(userId,jointType,orientation);
      buildJointArray(user, userList[i]);
      user++;
  PMatrix3D joint=new PMatrix3D();
  float confidence = context.getJointOrientationSkeleton(userList[i], SimpleOpenNI.SKEL_LEFT_SHOULDER, joint);
  joint.print();
    }      

    // draw the center of mass
    if (context.getCoM(userList[i], com))
    {
      context.convertRealWorldToProjective(com, com2d);
      stroke(100, 255, 0);
      strokeWeight(1);
      beginShape(LINES);
      vertex(com2d.x, com2d.y - 5);
      vertex(com2d.x, com2d.y + 5);

      vertex(com2d.x - 5, com2d.y);
      vertex(com2d.x + 5, com2d.y);
      endShape();

      fill(0, 255, 100);
      text(Integer.toString(userList[i]), com2d.x, com2d.y);
    }
  }
}

//head
//neck
//leftShoulder
//leftElbow
//leftHand
//rightShoulder
//rightElbow
//rightHand
//torso
//left hip
//left knee
//left foot
//right hip
//right knee
//right foot
void buildJointArray(int user, int userId)
{
  message = new OscMessage("/joints");
  message.add(user);
  PVector rootPos = new PVector();
  context.getJointPositionSkeleton(userId, SimpleOpenNI.SKEL_TORSO, rootPos);
  message.add(rootPos.x);
  message.add(rootPos.y);
  message.add(rootPos.z);
  jointIndex=4;
  addJoint(userId, SimpleOpenNI.SKEL_HEAD);
  addJoint(userId, SimpleOpenNI.SKEL_NECK);
  addJoint(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER);
  addJoint(userId, SimpleOpenNI.SKEL_LEFT_ELBOW);
  addJoint(userId, SimpleOpenNI.SKEL_LEFT_HAND);
  addJoint(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER);
  addJoint(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW);
  addJoint(userId, SimpleOpenNI.SKEL_RIGHT_HAND);
  addJoint(userId, SimpleOpenNI.SKEL_TORSO);
  addJoint(userId, SimpleOpenNI.SKEL_LEFT_HIP);
  addJoint(userId, SimpleOpenNI.SKEL_LEFT_KNEE);
  addJoint(userId, SimpleOpenNI.SKEL_LEFT_FOOT);
  addJoint(userId, SimpleOpenNI.SKEL_RIGHT_HIP);
  addJoint(userId, SimpleOpenNI.SKEL_RIGHT_KNEE);
  addJoint(userId, SimpleOpenNI.SKEL_RIGHT_FOOT);
  /* send the message */
  oscP5.send(message, myRemoteLocation); 
}

void addJoint(int userId, int jointName)
{
  PMatrix3D joint=new PMatrix3D();
  float confidence = context.getJointOrientationSkeleton(userId, jointName, joint);
  joints[jointIndex]=joint.m00;
  joints[jointIndex+1]=joint.m10;
  joints[jointIndex+2]=joint.m20;
  joints[jointIndex+3]=joint.m01;
  joints[jointIndex+4]=joint.m11;
  joints[jointIndex+5]=joint.m21;
  joints[jointIndex+6]=joint.m02;
  joints[jointIndex+7]=joint.m12;
  joints[jointIndex+8]=joint.m22;  
  
  for(int i=jointIndex;i<jointIndex+9;i++)
    message.add(joints[i]);
  jointIndex+=9;
}

// draw the skeleton with the selected joints
void drawSkeleton(int userId)
{
  // to get the 3d joint data
  /*
  PVector jointPos = new PVector();
   context.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_NECK,jointPos);
   println(jointPos);
   */
  stroke(255);
  strokeWeight(3);
  context.drawLimb(userId, SimpleOpenNI.SKEL_HEAD, SimpleOpenNI.SKEL_NECK);

  context.drawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_LEFT_SHOULDER);
  context.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_LEFT_ELBOW);
  context.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_ELBOW, SimpleOpenNI.SKEL_LEFT_HAND);

  context.drawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_RIGHT_SHOULDER);
  context.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_RIGHT_ELBOW);
  context.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW, SimpleOpenNI.SKEL_RIGHT_HAND);

  context.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_TORSO);
  context.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_TORSO);

  context.drawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_LEFT_HIP);
  context.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_HIP, SimpleOpenNI.SKEL_LEFT_KNEE);
  context.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_KNEE, SimpleOpenNI.SKEL_LEFT_FOOT);

  context.drawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_RIGHT_HIP);
  context.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_HIP, SimpleOpenNI.SKEL_RIGHT_KNEE);
  context.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_KNEE, SimpleOpenNI.SKEL_RIGHT_FOOT);
}

// -----------------------------------------------------------------
// SimpleOpenNI events

void onNewUser(SimpleOpenNI curContext, int userId)
{
  println("onNewUser - userId: " + userId);
  println("\tstart tracking skeleton");

  curContext.startTrackingSkeleton(userId);
}

void onLostUser(SimpleOpenNI curContext, int userId)
{
  println("onLostUser - userId: " + userId);
}

void onVisibleUser(SimpleOpenNI curContext, int userId)
{
  //println("onVisibleUser - userId: " + userId);
}


void keyPressed()
{
  switch(key)
  {
  case ' ':
    context.setMirror(!context.mirror());
    break;
  }
}  

