import processing.serial.*; 
 
Serial myPort;    // The serial port
int[] arr = new int[4096];
String s;
void setup() { 
  size(400,200); 

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
    println(inString.length);
    for(int i = 0; i < inString.length; i++) println("tag "+inString[i]+";");
    String[] list = split(inString[1], ",");
  for (int i=0 ;i<list.length;++i)
  {arr[i] =  Integer.parseInt(list[i]);} 
    
    s = null;
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