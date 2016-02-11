import processing.serial.*; 
 
Serial myPort;    // The serial port
int[] arr = new int[4096];
String s;
int maxy = 0;
void setup() { 
  size(800,600); 
frameRate(60);
  // List all the available serial ports: 
  printArray(Serial.list()); 
  // I know that the first port in the serial list on my mac 
  // is always my  Keyspan adaptor, so I open Serial.list()[0]. 
  // Open whatever port is the one you're using. 
  myPort = new Serial(this, Serial.list()[0], 256000); 
  myPort.bufferUntil('\n'); 
  
  
} 
 
void draw() { 

 background(0);
  stroke(255,255,0);
   fill(255);
   smooth();

line(0, height/2, width, height/2);

  if (s != null) {
    println(s);
    String[] inString = match(trim(s),"\\<(.*?)\\>\\[(.*?)\\]");
    println(inString[1]);
    String[] list = split(inString[2], ",");
  for (int i=0 ;i<list.length-1;++i) arr[i] =  Integer.parseInt(trim(list[i]));
  maxy = max(arr);
  println(maxy);
  println("ok");
    
    s = null;
  }
  
  
  for(int i = 1; i < 500; i++) {
    float Gx = (map(i, 1,500,0,width));
    float Gy = (map(arr[i], 0,500,0, height));
    line(Gx,0,Gx,Gy);
  }
  
} 
 
void serialEvent(Serial p) { 
s = p.readString();
// String[] inString = match(trim(s),"<(.*?)>[(.*?)]");
// for(int i = 0; i < inString.length; i++) println("tag "+inString[i]+";");
/* String[] list = split(inString[1], ",");
  for (int i=0 ;i<list.length;++i)
  {arr[i] =  Integer.parseInt(list[i]);} */
//  println(s);
  //println(inString[1]);
} 