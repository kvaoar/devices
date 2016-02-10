import processing.serial.*; 
 
Serial myPort;    // The serial port
int[] arr = new int[4096];
String s;
void setup() { 
  size(800,600); 

  // List all the available serial ports: 
  printArray(Serial.list()); 
  // I know that the first port in the serial list on my mac 
  // is always my  Keyspan adaptor, so I open Serial.list()[0]. 
  // Open whatever port is the one you're using. 
  myPort = new Serial(this, Serial.list()[0], 256000); 
  myPort.bufferUntil('\n'); 
  
  
} 
 
void draw() { 
  
  if (s != null) {
    String[] inString = match(trim(s),"\\<(.*?)\\>\\[(.*?)\\]");
    println(inString[1]);
    String[] list = split(inString[2], ",");
  for (int i=0 ;i<list.length-1;++i)
  {arr[i] =  Integer.parseInt(trim(list[i]));} 
  int maxy = max(arr);
  println(maxy);
  println("ok");
    
    s = null;
  }
  
  
  for(int i = 0; i < 4096; i++) {
    int Gx = round(map(i, 0,4096,0,width));
    int Gy = round(map(arr[i], 0,300,height,0));
    line(Gx,0,Gx,Gy);
  }
  
  background(0); 
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