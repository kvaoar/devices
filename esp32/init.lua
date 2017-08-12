
function receiver(sck, data)
  print(data)
  sck:close()
end

wifi.setmode(wifi.STATION);
wifi.start();
wifi.sta.config({ssid="HOMENET", pwd="lq@fp173"});
tmr.delay(10);
info = wifi.sta.getconfig();
count = 0;
timeout = false;
while(string.len(info.ssid) == 0) do    
  print(".");
  tmr.delay(1);
  info = wifi.sta.getconfig();
  count = count + 1;
  if(count == 3) then 
    print("Connect timeout");
    timeout = true;
    break; 
  end
end

if(timeout ~= true) then
  tmr.delay(2);
  print("run");
sv = net.createServer(net.TCP, 30)
sv:listen(80, function(conn)
  print("a")
end)

end