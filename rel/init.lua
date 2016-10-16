--require("ds18b20") 
--ds18b20.setup(4)
gpio.mode(0, gpio.OUTPUT)
gpio.write(0, gpio.LOW)

gpio.mode(5, gpio.OUTPUT)
gpio.write(5, gpio.LOW)
-----------------------
gpio.mode(6, gpio.OUTPUT)
gpio.write(6, gpio.LOW)
-----------------------
gpio.mode(7, gpio.OUTPUT)
gpio.write(7, gpio.LOW)
-----------------------
gpio.mode(8, gpio.OUTPUT)
gpio.write(8, gpio.LOW)

local function server_udp(client,data)
CMD = data
tmr.delay(10) 
--client:send ( CMD )
print("rxData: ",CMD)
tmr.delay(10)  

if     CMD=="A+" then
        gpio.write(5, gpio.HIGH)
elseif CMD=="A-" then
        gpio.write(5, gpio.LOW)
elseif CMD=="B+" then
        gpio.write(6, gpio.HIGH)
elseif CMD=="B-" then
        gpio.write(6, gpio.LOW)
elseif CMD=="C+" then
        gpio.write(7, gpio.HIGH)
elseif CMD=="C-" then
        gpio.write(7, gpio.LOW)
elseif CMD=="D+" then
        gpio.write(8, gpio.HIGH)
elseif CMD=="D-" then
        gpio.write(8, gpio.LOW)
elseif CMD=="on" then
        gpio.write(5, gpio.HIGH)
        gpio.write(6, gpio.HIGH)
        gpio.write(7, gpio.HIGH)
        gpio.write(8, gpio.HIGH)
elseif CMD=="off" then
        gpio.write(5, gpio.LOW)
        gpio.write(6, gpio.LOW)
        gpio.write(7, gpio.LOW)
        gpio.write(8, gpio.LOW)
elseif CMD=="t" then 
        require('ds18b20')
        ds18b20.setup(4) 
        local address = ds18b20.addrs()
        if(address ~= nil) then
        cu=net.createConnection(net.UDP,0)
        cu:connect(8080,"192.168.1.33") 
        cu:send(ds18b20.read())
        cu:close();   
        print(ds18b20.read()) 
        end
        ds18b20=nil
        package.loaded["ds18b20"]=nil
end      
end 
wifi.setmode(wifi.STATION)
wifi.sta.config("ESPNET","asdfgh123")
wifi.sta.connect()
i=0
tmr.alarm(1, 1000, 1, function()
    if (wifi.sta.status() ~= 5 ) then--and i < 10) then
       print("Status:"..wifi.sta.status())
       i = i + 1
       if( i > 1) then 
          i = 0
       end
       if (i == 0) then
           gpio.write(0, gpio.HIGH)
       else
           gpio.write(0, gpio.LOW)
       end
    else
       tmr.stop(1)
       if (wifi.sta.status() == 5) then
          print("IP:"..wifi.sta.getip())
          gpio.write(0, gpio.HIGH)
       else
          print("Status:"..wifi.sta.status())
       
              end
    end
end)
svr = net.createServer(net.UDP)
svr:on("receive", server_udp)
svr:listen (8080)
