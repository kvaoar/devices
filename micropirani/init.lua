local SSID = "***"
local SSID_PASSWORD = "***"
local DISABLED_CHARSET = "[\r\n\",:]";
local CMD = nil
local ANSWER = nil
-- udp server
local function server_udp(client,data)
CMD = string.gsub(data, DISABLED_CHARSET, "");  
ANSWER = nil;
uart.write(0, CMD.."\n")
cnt = 1
tmr.alarm (1, 300, 1, function ( )
    cnt = cnt + 1   
      if (ANSWER ~= nil) then
         tmr.stop (1)
         client:send ( "{\"CMD\": \""..CMD.."\", \"ANSW\": \""..ANSWER.."\"}" )
         ANSWER = nil;
      end
      if(cnt > 5) then 
          cnt = 0
          tmr.stop (1)
          client:send ( "{\"CMD\": \""..CMD.."\",\"ANSW\": \"".."TIMEOUT".."\"}" )
          ANSWER = nil;
      end
   end)
end
-- connect to wifi
function wait_for_wifi_conn ( )
   tmr.alarm (2, 1000, 1, function ( )
      if wifi.sta.getip ( ) ~= nil then
         tmr.stop (2)
         print(wifi.sta.getip())
      end
   end)
end
-- connect to usart
function usart_init()
gpio14 = 5;
gpio.mode(gpio14, gpio.INPUT, gpio.PULLUP)
if gpio.read(gpio14) == 1 then 
    uart.setup( 0, 9600, 8, 0, 1, 0 )
    print("uart on")
    uart.on("data", "\n", 
    function(data)
        ANSWER = string.gsub(data, DISABLED_CHARSET, "")
        print(ANSWER)
    end, 0)
end
end
-- Configure the ESP as station (client)
wifi.setmode (wifi.STATION)
wifi.sta.config (SSID, SSID_PASSWORD)
wifi.sta.autoconnect (1)
-- if we want change script - we cant use uart
usart_init()
-- Hang out until we get a wifi connection before the httpd server is started.
wait_for_wifi_conn ()
-- Create the udp server
svr = net.createServer (net.UDP)
svr:on("receive", server_udp)
svr:listen (8888)
