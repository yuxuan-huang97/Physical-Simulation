//Shallow Water Simulation

String projectTitle = "Shallow Water";

boolean play = false;

//Create Window
void setup() {
  size(600, 600, P3D);
  noStroke();
  //frameRate(32);
  for (int i = 0; i < n; i++){
    h[i] = 1;
    uh[i] = 0;
    hm[i] = 0;
    uhm[i] = 0;
  }
  uh[1] = 3;
  //for (int i = 2*n/5; i < 3*n/5; i++) h[i] = 3;
}

float g = 10;

int n = 50;
float dx = 0.1;
float[] h = new float[n];
float[] uh = new float[n];
//h[n/2] = 1;

float totlen = n*dx;
float[] hm = new float[n];
float[] uhm = new float[n];

void waveEquation(float dt){
  // halfstep
  for(int i = 0; i < n-1; i++){
    hm[i] = (h[i] + h[i+1])/2.0 - (dt/2.0)*(uh[i+1]-uh[i])/dx;
    uhm[i] = (uh[i] + uh[i+1])/2.0 - (dt/2.0)*
    ((uh[i+1]*uh[i+1])/h[i+1] + 0.5*g*h[i+1]*h[i+1] -
    (uh[i]*uh[i])/h[i] - 0.5*g*h[i]*h[i])/dx;
  }
  // fullstep
  float damp = 0.1;
  for(int i = 0; i < n-2; i++){
    h[i+1] -= dt*(uhm[i+1]-uhm[i])/dx;
    uh[i+1] -= dt*(damp*uh[i+1] + 
    (uhm[i+1]*uhm[i+1])/hm[i+1] + 0.5*g*hm[i+1]*hm[i+1] - 
    uhm[i]*uhm[i]/hm[i] - 0.5*g*hm[i]*hm[i])/dx;
  }
  // boundary conditions (Free)
  h[0] = h[n-2];
  uh[0] = uh[n-2];
  h[n-1] = h[1];
  uh[n-1] = uh[1];
}

//Draw the scene: one sphere per mass, one line connecting each pair
void draw() {
  background(255,255,255);
  //lights();
  ambientLight(128, 128, 128);
  directionalLight(128, 128, 128, 0, 1, 0);
  lightFalloff(1, 0, 0);
  lightSpecular(0, 0, 0);
  if (play) waveEquation(.002);
  fill(0, 0, 255);
  beginShape(QUADS);
  for (int i = 0; i < n-1; i++){
    vertex(0.5+600/n*i, 600, 0);
    vertex(0.5+600/n*i, 600-h[i]*100, 0); //<>//
    vertex(0.5+600/n*(i+0.5), 600-h[i]*100, 0);
    vertex(0.5+600/n*(i+0.5), 600, 0);
    vertex(0.5+600/n*(i+0.5), 600, 0);
    vertex(0.5+600/n*(i+0.5), 600-h[i]*100, 0);
    vertex(0.5+600/n*(i+1), 600-h[i+1]*100, 0);
    vertex(0.5+600/n*(i+1), 600, 0);
    
    vertex(0.5+600/n*i, 600-h[i]*100, 0);
    vertex(0.5+600/n*i, 600-h[i]*100, -200);
    vertex(0.5+600/n*(i+0.5), 600-h[i]*100, -200);    
    vertex(0.5+600/n*(i+0.5), 600-h[i]*100, 0);

    vertex(0.5+600/n*(i+0.5), 600-h[i]*100, 0);
    vertex(0.5+600/n*(i+0.5), 600-h[i]*100, -200);
    vertex(0.5+600/n*(i+1), 600-h[i+1]*100, -200);
    vertex(0.5+600/n*(i+1), 600-h[i+1]*100, 0);

  }
  endShape();
  
  String runtimeReport = 
        " FPS: "+ str(round(frameRate)) +"\n";
  surface.setTitle(projectTitle+ "  -  " +runtimeReport);
}

void keyPressed() {
  if (keyCode == ENTER) play = true;
}
