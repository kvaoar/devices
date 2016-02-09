import processing.serial.*; 
 
Serial myPort;    // The serial port
int[] arr = new int[4096];
 
void setup() { 
  size(400,200); 

  // List all the available serial ports: 
  printArray(Serial.list()); 
  // I know that the first port in the serial list on my mac 
  // is always my  Keyspan adaptor, so I open Serial.list()[0]. 
  // Open whatever port is the one you're using. 
  myPort = new Serial(this, Serial.list()[1], 9600); 
  myPort.bufferUntil('\n'); 
} 
 
void draw() { 
  background(0); 
} 
 
void serialEvent(Serial p) { 
 String[] inString = match(p.readString(),"<(.*?)>[(.*?)]");
 String[] list = split(inString[1], ",");
  for (int i=0 ;i<list.length;++i)
  {arr[i] = ;} 
} 