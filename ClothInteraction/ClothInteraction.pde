//Triple Spring (damped) - 1D Motion
//CSCI 5611 Thread Sample Code
// Stephen J. Guy <sjguy@umn.edu>
// Modified by Yuxuan Huang into a simulated cloth

String projectTitle = "Simulated Cloth";

//Create Window
void setup() {
  size(600, 600, P3D);
  noStroke();
  camera = new Camera();
  img = loadImage("texture.jpg");
  frameRate(32);
}

PImage img;

//Simulation Parameters
float floor = 500;
float gravity = 10;
//float radius = 5;
//float stringTop = 50;
float restLen = 5;
float mass = 5; //TRY-IT: How does changing mass affect resting length?
float k = 1000; //TRY-IT: How does changing k affect resting length?
float kv = 100;

float time = 0;

float anchorX = 200;
float anchorY = 100;
float anchorZ = -200;
float disT = 5;
float disX = 0;
float disY = 5;
float disZ = 0;

int numofbead = 30;
int numofthread = 30;

float radius = 50;
float spherex = 320;
float spherey = 200;
float spherez = -105;

boolean play = true;
float delta_x = 0;
boolean mm = false;

class Bead {
  PVector pos = new PVector();
  PVector vel = new PVector();
  public Bead(float x, float y, float z, float vx, float vy, float vz) {
    pos.x = x;
    pos.y = y;
    pos.z = z;
    vel.x = vx;
    vel.y = vy;
    vel.z = vz;
    //println(pos.z);
  }
}

class MyThread {
  Bead[] b = new Bead[numofbead];
  float[] f = new float[numofbead];
  public MyThread(float x, float y, float z, float deltax, float delta_y, float delta_z) {
    for (int i = 0; i < numofbead; i++) {
      b[i] = new Bead(x + (i + 1) * deltax , y + (i + 1) * delta_y , z + (i + 1) * delta_z, 0, 0, 0);
      //println(x + i * deltax, " ",y + (i + 1) * delta_y);
    }
  }
}

class Cloth {
  MyThread[] t = new MyThread[numofthread];
  public Cloth(float x, float y, float z, float delta_t, float delta_x, float delta_y, float delta_z) {
    for (int i = 0; i < numofthread; i++) {
      t[i] = new MyThread(x + i * delta_t, y, z, delta_x, delta_y, delta_z); // default no displacement in z direction
    }
  }
}




Cloth c = new Cloth(anchorX, anchorY, anchorZ, disT, disX, disY, disZ);

void update(float dt){
  
  float stringF;
  float dampF;

  float[] forceXv = new float[numofthread * numofbead];
  float[] forceYv = new float[numofthread * numofbead];
  float[] forceZv = new float[numofthread * numofbead];
  
  float[] forceXh = new float[(numofthread-1) * numofbead];
  float[] forceYh = new float[(numofthread-1) * numofbead];
  float[] forceZh = new float[(numofthread-1) * numofbead];
  
  PVector strdir; // string direction
  float v1;
  float v2;
  
  for (int t = 0; t < 40; t++) {
    //Compute (damped) Hooke's law for each spring vertical
    for (int i = 0; i < numofthread; i++) { // each thread
      for (int j = 0; j < numofbead; j++) { // each bead
        float deltax;
        float deltay;
        float deltaz;
        float len;
        if (j == 0) {
          deltax = c.t[i].b[j].pos.x - (anchorX + i*disT);
          deltay = c.t[i].b[j].pos.y - anchorY;
          deltaz = c.t[i].b[j].pos.z - anchorZ;
        }
        else {
          deltax = c.t[i].b[j].pos.x - c.t[i].b[j-1].pos.x;
          deltay = c.t[i].b[j].pos.y - c.t[i].b[j-1].pos.y;
          deltaz = c.t[i].b[j].pos.z - c.t[i].b[j-1].pos.z;
        } //<>//
        len = sqrt(deltax*deltax + deltay*deltay + deltaz*deltaz);
        strdir = new PVector(deltax, deltay, deltaz);
        strdir.normalize();
        v1 = c.t[i].b[j].vel.dot(strdir);
        if (j == 0) v2 = 0;
        else v2 = c.t[i].b[j-1].vel.dot(strdir);
        dampF = -kv * (v1 - v2);
        stringF = -k*(len - restLen);
        forceXv[numofbead * i + j] = stringF * deltax / len + dampF * deltax / len; 
        forceYv[numofbead * i + j] = stringF * deltay / len + dampF * deltay / len;
        forceZv[numofbead * i + j] = stringF * deltaz / len + dampF * deltaz / len;
      }
    }
    
    // horizontal
    for (int i = 0; i < numofthread-1; i++) { // each thread
      for (int j = 0; j < numofbead; j++) { // each bead
        float deltax;
        float deltay;
        float deltaz;
        float len;
        
        deltax = c.t[i].b[j].pos.x - c.t[i+1].b[j].pos.x;
        deltay = c.t[i].b[j].pos.y - c.t[i+1].b[j].pos.y;
        deltaz = c.t[i].b[j].pos.z - c.t[i+1].b[j].pos.z;
        
        len = sqrt(deltax*deltax + deltay*deltay + deltaz*deltaz);
        //println(len);
        strdir = new PVector(deltax, deltay, deltaz);
        strdir.normalize();
        v1 = c.t[i].b[j].vel.dot(strdir);
        v2 = c.t[i+1].b[j].vel.dot(strdir);
        dampF = -kv * (v1 - v2);
        //dampF = 0;
        stringF = -k*(len - restLen);
        //println(dampF, " ", stringF);
        forceXh[numofbead * i + j] = stringF * deltax / len + dampF * deltax / len;
        forceYh[numofbead * i + j] = stringF * deltay / len + dampF * deltay / len;
        forceZh[numofbead * i + j] = stringF * deltaz / len + dampF * deltaz / len;
      }
    } 
    
    
    //Eulerian integration
    float acc_x;
    float acc_y;
    float acc_z;
    for (int i = 0; i < numofthread; i++) { // each thread
      for (int j = 0; j < numofbead; j++) { // each bead
        acc_x = .5*forceXv[numofbead * i + j]/mass;
        acc_y = gravity + .5*forceYv[numofbead * i + j]/mass;
        acc_z = .5*forceZv[numofbead * i + j]/mass;
        if (j < numofbead - 1) { // pulled by two strings
          acc_x -= .5*forceXv[numofbead * i + j + 1]/mass;
          acc_y -= .5*forceYv[numofbead * i + j + 1]/mass;
          acc_z -= .5*forceZv[numofbead * i + j + 1]/mass;
        }
        
        if (i != 0) {
          acc_x -= .5*forceXh[numofbead * (i-1) + j]/mass;
          acc_y -= .5*forceYh[numofbead * (i-1) + j]/mass;
          acc_z -= .5*forceZh[numofbead * (i-1) + j]/mass;
        }
        if (i != numofthread - 1) {
          acc_x += .5*forceXh[numofbead * (i) + j]/mass;
          acc_y += .5*forceYh[numofbead * (i) + j]/mass;
          acc_z += .5*forceZh[numofbead * (i) + j]/mass;
        }
        
        //println(acc_z);
        c.t[i].b[j].vel.x += acc_x * dt;
        c.t[i].b[j].pos.x += c.t[i].b[j].vel.x * dt;
        c.t[i].b[j].vel.y += acc_y * dt;
        c.t[i].b[j].pos.y += c.t[i].b[j].vel.y * dt;
        c.t[i].b[j].vel.z += acc_z * dt;
        c.t[i].b[j].pos.z += c.t[i].b[j].vel.z * dt;
      }
    }
    
    //Collision detection and response
    
    float alpha = 0.5;
    if (mm) {
      spherez += delta_x;
      mm = false;
    }
    for (int i = 0; i < numofthread; i++) { // each thread
      for (int j = 0; j < numofbead; j++) { // each bead
        if ((c.t[i].b[j].pos.x-spherex)*(c.t[i].b[j].pos.x-spherex) +
        (c.t[i].b[j].pos.y-spherey)*(c.t[i].b[j].pos.y-spherey) +
        (c.t[i].b[j].pos.z-spherez)*(c.t[i].b[j].pos.z-spherez) <= (radius+0.1)*(radius+0.1)) {
          PVector n = new PVector(c.t[i].b[j].pos.x-spherex, c.t[i].b[j].pos.y-spherey,
          c.t[i].b[j].pos.z-spherez);
          n.normalize();
          float vns = PVector.dot(c.t[i].b[j].vel, n); // velocity parallel to normal (scalar)
          //println(vns);
          PVector vtemp = PVector.mult(n, -(1+alpha)*vns);
          c.t[i].b[j].vel.add(vtemp);
          n.mult(radius + 1);
          c.t[i].b[j].pos.x = spherex + n.x;
          c.t[i].b[j].pos.y = spherey + n.y;
          c.t[i].b[j].pos.z = spherez + n.z;
          //println((c.t[i].b[j].pos.x-spherex)*(c.t[i].b[j].pos.x-spherex) +
        //(c.t[i].b[j].pos.y-spherey)*(c.t[i].b[j].pos.y-spherey) +
        //(c.t[i].b[j].pos.z-spherez)*(c.t[i].b[j].pos.z-spherez) - radius*radius);
        }
      }
    }
    
  }
}

//Draw the scene: one sphere per mass, one line connecting each pair
void draw() {
  background(0,0,0);
  lights();
  camera.Update( 1.0/frameRate );
  if (play) update(.005); //We're using a fixed, large dt -- this is a bad idea!!
  
  
  //beginShape(QUADS);
  textureMode(IMAGE);
  for (int i = 0; i < numofthread-1; i++) { // each thread
  //for (int i = 0; i < numofthread; i++) { // each thread
    for (int j = 0; j < numofbead; j++) { // each bead     
      
      beginShape(TRIANGLES);
      texture(img);
      if (j == 0) {
        vertex(anchorX + i*disT, anchorY, anchorZ, 17*i, 0);
        vertex(anchorX + (i+1)*disT, anchorY, anchorZ, 17*(i+1), 0);
      }
      else {
        vertex(c.t[i].b[j-1].pos.x, c.t[i].b[j-1].pos.y, c.t[i].b[j-1].pos.z, 17*i, 17*(j-1));
        vertex(c.t[i+1].b[j-1].pos.x, c.t[i+1].b[j-1].pos.y, c.t[i+1].b[j-1].pos.z, 17*(i+1), 17*(j-1));
      }
      vertex(c.t[i+1].b[j].pos.x, c.t[i+1].b[j].pos.y, c.t[i+1].b[j].pos.z, 17*(i+1), 17*j);
      endShape();
      beginShape(TRIANGLES);
      texture(img);
      vertex(c.t[i].b[j].pos.x, c.t[i].b[j].pos.y, c.t[i].b[j].pos.z, 17*i, 17*j);
      vertex(c.t[i+1].b[j].pos.x, c.t[i+1].b[j].pos.y, c.t[i+1].b[j].pos.z, 17*(i+1), 17*j);
      if (j == 0) vertex(anchorX + i*disT, anchorY, anchorZ, 17*i, 0);
      else vertex(c.t[i].b[j-1].pos.x, c.t[i].b[j-1].pos.y, c.t[i].b[j-1].pos.z, 17*i, 17*(j-1));
      endShape();
    }
  }
  //endShape();
  
  fill(255, 255, 255);
  pushMatrix();
  translate(spherex, spherey, spherez);
  sphere(radius);
  popMatrix();
  
  String runtimeReport = 
        " FPS: "+ str(round(frameRate)) +"\n";
  surface.setTitle(projectTitle+ "  -  " +runtimeReport);
}






// ************** Camera Setting ******************
class Camera
{
  Camera()
  {
    
    position      = new PVector( 34.296555,   70.19721,   52.12865 ); // initial position
    theta         = -0.9; // rotation around Y axis. Starts with forward direction as ( 0, 0, -1 )
    phi           = -0.27; // rotation around X axis. Starts with up direction as ( 0, 1, 0 )
    
    /*
    position      = new PVector(66.88244,   84.58544,   -161.01736); // initial position
    theta         = -1.739; // rotation around Y axis. Starts with forward direction as ( 0, 0, -1 )
    phi           = -0.27; // rotation around X axis. Starts with up direction as ( 0, 1, 0 )
    */
    moveSpeed     = 50;
    turnSpeed     = 1.57; // radians/sec
    
    // dont need to change these
    negativeMovement = new PVector( 0, 0, 0 );
    positiveMovement = new PVector( 0, 0, 0 );
    negativeTurn     = new PVector( 0, 0 ); // .x for theta, .y for phi
    positiveTurn     = new PVector( 0, 0 );
    fovy             = PI / 4;
    aspectRatio      = width / (float) height;
    nearPlane        = 0.1;
    farPlane         = 10000;
  }
  
  void Update( float dt )
  {
    theta += turnSpeed * (negativeTurn.x + positiveTurn.x) * dt;
    
    // cap the rotation about the X axis to be less than 90 degrees to avoid gimble lock
    float maxAngleInRadians = 85 * PI / 180;
    phi = min( maxAngleInRadians, max( -maxAngleInRadians, phi + turnSpeed * ( negativeTurn.y + positiveTurn.y ) * dt ) );
    
    // re-orienting the angles to match the wikipedia formulas: https://en.wikipedia.org/wiki/Spherical_coordinate_system
    // except that their theta and phi are named opposite
    float t = theta + PI / 2;
    float p = phi + PI / 2;
    PVector forwardDir = new PVector( sin( p ) * cos( t ),   cos( p ),   -sin( p ) * sin ( t ) );
    PVector upDir      = new PVector( sin( phi ) * cos( t ), cos( phi ), -sin( t ) * sin( phi ) );
    PVector rightDir   = new PVector( cos( theta ), 0, -sin( theta ) );
    PVector velocity   = new PVector( negativeMovement.x + positiveMovement.x, negativeMovement.y + positiveMovement.y, negativeMovement.z + positiveMovement.z );
    position.add( PVector.mult( forwardDir, moveSpeed * velocity.z * dt ) );
    position.add( PVector.mult( upDir,      moveSpeed * velocity.y * dt ) );
    position.add( PVector.mult( rightDir,   moveSpeed * velocity.x * dt ) );
    
    aspectRatio = width / (float) height;
    perspective( fovy, aspectRatio, nearPlane, farPlane );
    camera( position.x, position.y, position.z,
            position.x + forwardDir.x, position.y + forwardDir.y, position.z + forwardDir.z,
            upDir.x, upDir.y, upDir.z );
    //println(position.x, " ", position.y, " ", position.z, " ", theta, " ", phi);
  }
  
  // only need to change if you want difrent keys for the controls
  void HandleKeyPressed()
  {
    if ( key == 'w' ) positiveMovement.z = 10;
    if ( key == 's' ) negativeMovement.z = -10;
    if ( key == 'a' ) negativeMovement.x = -10;
    if ( key == 'd' ) positiveMovement.x = 10;
    if ( key == 'q' ) positiveMovement.y = 10;
    if ( key == 'e' ) negativeMovement.y = -10;
    
    if ( keyCode == LEFT )  negativeTurn.x = 1;
    if ( keyCode == RIGHT ) positiveTurn.x = -1;
    if ( keyCode == UP )    positiveTurn.y = 1;
    if ( keyCode == DOWN )  negativeTurn.y = -1;
  }
  
  // only need to change if you want difrent keys for the controls
  void HandleKeyReleased()
  {
    if ( key == 'w' ) positiveMovement.z = 0;
    if ( key == 'q' ) positiveMovement.y = 0;
    if ( key == 'd' ) positiveMovement.x = 0;
    if ( key == 'a' ) negativeMovement.x = 0;
    if ( key == 's' ) negativeMovement.z = 0;
    if ( key == 'e' ) negativeMovement.y = 0;
    
    if ( keyCode == LEFT  ) negativeTurn.x = 0;
    if ( keyCode == RIGHT ) positiveTurn.x = 0;
    if ( keyCode == UP    ) positiveTurn.y = 0;
    if ( keyCode == DOWN  ) negativeTurn.y = 0;
  }
  
    // only necessary to change if you want different start position, orientation, or speeds
  PVector position;
  float theta;
  float phi;
  float moveSpeed;
  float turnSpeed;
  
  // probably don't need / want to change any of the below variables
  float fovy;
  float aspectRatio;
  float nearPlane;
  float farPlane;  
  PVector negativeMovement;
  PVector positiveMovement;
  PVector negativeTurn;
  PVector positiveTurn;
};

Camera camera;

void keyPressed()
{
  camera.HandleKeyPressed();
  if (keyCode == ENTER) play = true;
}

void keyReleased()
{
  camera.HandleKeyReleased();
}

void mouseDragged() {
  mm = true;
  delta_x = mouseX - pmouseX;
}

// **********************************************************************
