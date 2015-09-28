import hypermedia.net.*; //<>// //<>//
UDP udp; // define the UDP object
float sw, sh, touchX, touchY;
FloatList answer;
int time = 0;
int wait = 1000;
int pow = 1;
boolean no_answ = true;
String message = "@253PR1?;FF"; // the message to send
String ESP_ip = "255.255.255.255"; // the remote IP address
int ESP_port = 8888; // the destination port

void setup() {
size(displayWidth,displayHeight, P2D);
println(displayWidth);
println(displayHeight);
sw = displayWidth;
sh = displayHeight;
udp = new UDP( this, 6000 );
//udp.log( true ); // <— printout the connection activity
udp.listen( true );
udp.broadcast( true );
time = millis();//store the current time
answer = new FloatList();

}

// Calculates the base-10 logarithm of a number
float log10 (float x) {
return (log(x) / log(10));
}

void draw() {
background(0);
smooth();
if( no_answ)
fill(255,0,0);
else
fill(255);

stroke(255,0,0);
strokeWeight(2);
ellipse(touchX, touchY, sw/100, sw/100);
textSize(48);
if(answer.size() > 2){
if (answer.size() > sw) answer.clear();
text("ANSWER: "+answer.get(answer.size()-1), sw/2-200,sh/2);


stroke(255,0,0);
for(int i = 1; i < answer.size(); i++) {
float y0 = 5+log10(answer.get(i-1));
float y1 = 5+log10(answer.get(i));
strokeWeight(5);
line(i-1, (sh/9)*y0, i, (sh/9)*y1);
}


stroke(255,255,0);
for(int i = 0; i < 9; i++) {
line(0, (sh/9)*i, sw, (sh/9)*i);
text("10^"+(i-5),10,(sh/9)*i+30);
}

}

if(millis() - time >= wait){
udp.send( message, ESP_ip, ESP_port );
no_answ = true;
time = millis();//also update the stored time
}




}

void mouseDragged() {
touchX = mouseX;
touchY = mouseY;
}

void mousePressed(){

}

void receive( byte[] data, String ip, int port ) {

no_answ = false;
ESP_ip = ip;
data = subset(data, 0, data.length);
String s = new String( data );
String[] res = match(s, "ACK(.*?);FF");
if( res != null) answer.append( Float.parseFloat(res[1]));

}