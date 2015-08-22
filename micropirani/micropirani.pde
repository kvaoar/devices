import hypermedia.net.*; //<>//

String ip       = "192.168.1.102";  // the remote IP address
int port        = 8888;    // the destination port
UDP udp;  // define the UDP object

 JSONObject json;
String CMD, ANSW;

void setup() { 
   udp = new UDP( this, 6000 );
     udp.listen( true );
  size(800,600); 
  frameRate(60);
  
} 
 
void draw() { 

 background(0);
  stroke(255,255);
   fill(255);
   smooth();
    text("CMD "+ CMD , 20,10);
    text("ANSW "+ANSW, 20,40);
} 


void receive( byte[] data, String ip, int port ) {
  data = subset(data, 0, data.length);
  String message = new String( data );
  println( "receive: \""+message+"\" from "+ip+" on port "+port );
  
  json = JSONObject.parse(message);
  CMD = json.getString("CMD");
  ANSW = json.getString("ANSW");
}


void keyReleased(){
    String  message = "jhgvfujgfvguh;\n";
    udp.send( message, ip, port ); 
 //<>// //<>// //<>//

}