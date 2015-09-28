local DISABLED_CHARSET = "[\r\n\",:]"
local CMD = nil
local ANSWER = "" 
-- udp server 
local function server_udp(client,data)
    CMD = string.gsub(data, DISABLED_CHARSET, "")
    uart.write(0, CMD.."\n")
    tmr.delay(1000) 
    client:send ( ANSWER )
    ANSWER = ""
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

--init uart
function usart_init()
gpio14 = 5;
gpio.mode(gpio14, gpio.INPUT, gpio.PULLUP)
if gpio.read(gpio14) == 1 then 
    uart.setup( 0, 9600, 8, 0, 1, 0 )
    print("uart on")
    uart.on("data", 0, 
    function(data)
    ANSWER = ANSWER..string.gsub(data, DISABLED_CHARSET, "")
    end, 0)
end
end

local str=wifi.ap.getmac()
local ssidTemp=string.format("%s%s%s",string.sub(str,10,11),string.sub(str,13,14),string.sub(str,16,17))
cfg={}
cfg.ssid="ESP8266_"..ssidTemp
cfg.pwd="12345678"
wifi.ap.config(cfg)
cfg={}
cfg.ip="192.168.1.1"
cfg.netmask="255.255.255.0"
cfg.gateway="192.168.1.1"
wifi.ap.setip(cfg)
wifi.setmode(wifi.SOFTAP)
collectgarbage();
usart_init()
svr = net.createServer (net.UDP)
svr:on("receive", server_udp)
svr:listen (8888)
