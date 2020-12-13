#include  <SoftwareSerial.h>

#define X_PIN   A0
#define Y_PIN   A1
#define COUNT   20

SoftwareSerial  bt = SoftwareSerial (2, 3);
int vx, vy;
int xoff, yoff;

void setup() {
  int i;
  Serial.begin(9600);
  bt.begin(9600);
  pinMode( X_PIN, INPUT );
  pinMode( Y_PIN, INPUT );
  Serial.println("Calibration started");
  xoff = 0; yoff = 0;
  for ( i=0; i<COUNT; i++ ) {
    vx = analogRead(X_PIN);
    vy = analogRead(Y_PIN);
    delay(200);
    xoff += vx;
    yoff += vy;
  }
  xoff = xoff / COUNT;
  yoff = yoff / COUNT;
  Serial.println("Calibration completed");
  Serial.println(xoff);
  Serial.println(yoff);
}

void getXY(int *px, int *py)
{
  *px = analogRead(X_PIN) - xoff;
  *py = analogRead(Y_PIN) - yoff;
  return;
}

char getCommand()
{
  char cmd;
  
  getXY(&vx, &vy);
  // Serial.println(vx);
  // Serial.println(vy);
  cmd = 'S';
  if(vx<-200)
    cmd='B';
  else if (vx>200)
    cmd='F';
  else if (vy<-200)
    cmd='L';
  else if (vy>200)
    cmd='R';
   
  Serial.println(cmd);
  
    
  
  return cmd;
  
}

void loop() {
  char cmd;
  char oldcmd = '?';

  do
  {
  cmd = getCommand();
    if(cmd != oldcmd)
    {
      oldcmd = cmd;
      bt.write(cmd);
    }
    delay(50);
  } while ( cmd != '#');
} 
