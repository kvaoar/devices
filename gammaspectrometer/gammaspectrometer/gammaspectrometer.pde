import processing.serial.*; 
 
Serial myPort;    // The serial port
int[] arrDIF = new int[4096];
int[] arrSUM = new int[4096];
float[] lin_arrDIF;
float[] lin_arrSUM;

int zoom = 1;
int partrate = 0;
int parttotal = 0;
String s;
int maxy = 10;
void setup() { 
  size(1024,600); 
   lin_arrDIF = new float[width+1];
   lin_arrSUM = new float[width+1];
frameRate(60);
  // List all the available serial ports: 
  printArray(Serial.list()); 
  // I know that the first port in the serial list on my mac 
  // is always my  Keyspan adaptor, so I open Serial.list()[0]. 
  // Open whatever port is the one you're using. 
  myPort = new Serial(this, Serial.list()[1], 256000); 
  myPort.bufferUntil('\n'); 
  
  
} 

void rescale(int arr[], float lin_arr[]){
     for(int i = 0; i < width; i++) lin_arr[i] = 0;
     int scaleX = 4096/width; 
     for(int i = 0; i < width; i++) for(int j = 1; j < scaleX; j++) lin_arr[i] += arr[i*scaleX+j];
     float lin_max = max(lin_arr);
     for(int i = 0; i < width; i++) lin_arr[i] *= ((0.4*height)/lin_max);
}
 
void draw() { 

 background(0);
  stroke(255,255,255);
   fill(255,255,255);
   smooth();


  if (s != null) {
      //println(s);
      try{
        String[] inString = match(trim(s),"\\<(.*?)\\>\\[(.*?)\\]");
       // println(unhex(inString[1]));
        partrate = unhex(inString[1]);
        parttotal += partrate;

        String[] list = split(inString[2], ",");
        for (int i=0 ;i<list.length;++i) {
          arrDIF[i] =  unhex(list[i]); 
          arrSUM[i] += unhex(list[i]); 
        }
      }
      catch(Exception e) { s = null;};
      s = null;
     // arr[0] = 0;
     // arr[4095] = 0;
      
      rescale(arrSUM,lin_arrSUM);
      rescale(arrDIF,lin_arrDIF);
        

  }
  
stroke(255,0,255);
 for(int i = 0; i < width; i++) line(i, height-16, i,  height-16-lin_arrSUM[i]);
 stroke(255,0,0);
 for(int i = 0; i < width; i++) line(i, (height/2)-16, i,  (height/2)-16-lin_arrDIF[i]);
 
 stroke(255,255,255);
 line(0,height/2,width,height/2);
 line(0,height/2 - 16,width,height/2 - 16);
 line(0,height,width,height);
 line(0,height - 16,width,height - 16);
 for(int i = 0; i < 20; i++) {
   float Eline = 2*26*i;
   String Evalue;
   if(i*200 < 1000) Evalue = str(i*200)+"k";
   else Evalue = nf(i*0.2,1,1)+"M";
   
textSize(14);
textAlign(LEFT);
 text(Evalue,Eline+5,(height/2)-2);
 line(Eline,(height/2),Eline,(height/2)-16);
 text(Evalue,Eline+5,height-2);
 line(Eline,height,Eline,height-16);
}

textSize(16);
textAlign(LEFT);
text(str(partrate)+"p/s",width-100,20);
text(str(parttotal)+"prt",width-100,40); 
  
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

void mouseWheel(MouseEvent event) {
  float e = event.getCount();
  println(e);
}