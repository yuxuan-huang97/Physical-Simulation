//Triple Spring (damped) - 1D Motion
//CSCI 5611 Thread Sample Code
// Stephen J. Guy <sjguy@umn.edu>
// Modified by Yuxuan Huang into a simulated cloth

//Create Window
void setup() {
  size(600, 600, P3D);
  surface.setTitle("Thread of Cloth");
}
  

//Simulation Parameters
float floor = 500;
float gravity = 10;
float radius = 10;
float stringTop = 50;
float restLen = 20;
float mass = 10; //TRY-IT: How does changing mass affect resting length?
float k = 500; //TRY-IT: How does changing k affect resting length?
float kv = 30;

float time = 0;

float anchorX = 200;
float anchorY = 200;
float disT = 100;
float disX = 18;
float disY = 5;

int numofbead = 10;
int numofthread = 3;

class Bead {
  PVector pos = new PVector();
  PVector vel = new PVector();
  public Bead(float x, float y, float vx, float vy) {
    pos.x = x;
    pos.y = y;
    vel.x = vx;
    vel.y = vy;
  }
}

class MyThread {
  Bead[] b = new Bead[numofbead];
  float[] f = new float[numofbead];
  public MyThread(float x, float y, float deltax, float delta_y) {
    for (int i = 0; i < numofbead; i++) {
      b[i] = new Bead(x + (i + 1) * deltax , y + (i + 1) * delta_y , 0, 0);
      println(x + i * deltax, " ",y + (i + 1) * delta_y);
    }
  }
}

class Cloth {
  MyThread[] t = new MyThread[numofthread];
  public Cloth(float x, float y, float delta_t, float delta_x, float delta_y) {
    for (int i = 0; i < numofthread; i++) {
      t[i] = new MyThread(x + i * delta_t, y, delta_x, delta_y);
    }
  }
}




Cloth c = new Cloth(anchorX, stringTop, disT, disX, disY);
/*
//Inital positions and velocities of masses
float ballY1 = 200;
float velY1 = 0;
float ballY2 = 250;
float velY2 = 0;
float ballY3 = 300;
float velY3 = 0;
*/

void update(float dt){
  
  /*
  float stringF;
  float dampF;
  float[] forceY = new float[3 * 3];
  
  //Compute (damped) Hooke's law for each spring
  for (int i = 0; i < 3; i++) { // each thread
    for (int j = 0; j < 3; j++) { // each bead
      if (j == 0) {
        stringF = -k * ((c.t[i].b[j].pos.y - stringTop) - restLen);
        dampF = -kv*(c.t[i].b[j].vel.y - 0);
      }
      else {
        stringF = -k * ((c.t[i].b[j].pos.y - c.t[i].b[j-1].pos.y) - restLen);
        dampF = -kv*(c.t[i].b[j].vel.y - c.t[i].b[j-1].vel.y);
      }
      forceY[3 * i + j] = stringF + dampF;
    }
  }
  */
  
  
  float stringF;
  float dampF;
  //float dampFX;
  //float dampFY;
  float[] forceX = new float[numofthread * numofbead];
  float[] forceY = new float[numofthread * numofbead];
  PVector strdir; // string direction
  float v1;
  float v2;
  
  for (int t = 0; t < 20; t++) {
    //Compute (damped) Hooke's law for each spring
    for (int i = 0; i < numofthread; i++) { // each thread
      for (int j = 0; j < numofbead; j++) { // each bead
        float deltax;
        float deltay;
        float len;
        if (j == 0) {
          deltax = c.t[i].b[j].pos.x - (anchorX + i*disT);
          deltay = c.t[i].b[j].pos.y - stringTop;
          //dampFX = -kv*(c.t[i].b[j].vel.x - 0);
          //dampFY = -kv*(c.t[i].b[j].vel.y - 0);
        }
        else {
          deltax = c.t[i].b[j].pos.x - c.t[i].b[j-1].pos.x;
          deltay = c.t[i].b[j].pos.y - c.t[i].b[j-1].pos.y;
          //dampFX = -kv*(c.t[i].b[j].vel.x - c.t[i].b[j-1].vel.x);
          //dampFY = -kv*(c.t[i].b[j].vel.y - c.t[i].b[j-1].vel.y);
        }
        len = sqrt(deltax*deltax + deltay*deltay);
        strdir = new PVector(deltax, deltay);
        v1 = c.t[i].b[j].vel.dot(strdir);
        if (j == 0) v2 = 0;
        else v2 = c.t[i].b[j-1].vel.dot(strdir);
        dampF = -kv * (v1 - v2);
        stringF = -k*(len - restLen);
        forceX[numofbead * i + j] = stringF * deltax / len + dampF * deltax / len; 
        forceY[numofbead * i + j] = (stringF * deltay / len) + dampF * deltay / len;
      }
    }
    
    
    
    //Eulerian integration
    float acc_x;
    float acc_y;
    for (int i = 0; i < numofthread; i++) { // each thread
      for (int j = 0; j < numofbead; j++) { // each bead
        acc_x = .5*forceX[numofbead * i + j]/mass;
        acc_y = gravity + .5*forceY[numofbead * i + j]/mass;
        if (j < numofbead - 1) { // pulled by two strings
          acc_x -= .5*forceX[numofbead * i + j + 1]/mass;
          acc_y -= .5*forceY[numofbead * i + j + 1]/mass;
        }
        c.t[i].b[j].vel.x += acc_x * dt;
        c.t[i].b[j].pos.x += c.t[i].b[j].vel.x * dt;
        c.t[i].b[j].vel.y += acc_y * dt;
        c.t[i].b[j].pos.y += c.t[i].b[j].vel.y * dt;
      }
    }
    
    //Collision detection and response
    for (int i = 0; i < numofthread; i++) { // each thread
      for (int j = 0; j < numofbead; j++) { // each bead
        if (c.t[i].b[j].pos.y + radius > floor) {
          c.t[i].b[j].vel.y *= -.9;
          c.t[i].b[j].pos.y = floor - radius;
        }
      }
    }
  }
}

//Draw the scene: one sphere per mass, one line connecting each pair
void draw() {
  background(255,255,255); //<>//
  update(.01); //We're using a fixed, large dt -- this is a bad idea!!
  //println(frameRate);
  //update(1.0/frameRate);
  //update((millis() - time)/ 1000.0);
  //time = millis();
  fill(0,0,0);
  
  for (int i = 0; i < numofthread; i++) { // each thread
    for (int j = 0; j < numofbead; j++) { // each bead
      pushMatrix();
      if (j == 0) line(anchorX + i*disT, stringTop, c.t[i].b[j].pos.x, c.t[i].b[j].pos.y);
      else line(c.t[i].b[j-1].pos.x, c.t[i].b[j-1].pos.y, c.t[i].b[j].pos.x, c.t[i].b[j].pos.y);
      //if (i < 2) line(c.t[i].b[j].pos.x, c.t[i].b[j].pos.y, c.t[i+1].b[j].pos.x, c.t[i+1].b[j].pos.y);
      translate(c.t[i].b[j].pos.x, c.t[i].b[j].pos.y);
      sphere(radius);
      popMatrix();
    }
  }
}
