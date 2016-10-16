-- Set module name as parameter of require
local modname = ...
local M = {}
_G[modname] = M
local pin = nil
local defaultPin = 9
local table = table
local string = string
local ow = ow
local tmr = tmr
setfenv(1,M)

function setup(dq)
  pin = dq
  if(pin == nil) then
    pin = defaultPin
  end
  ow.setup(pin)
end

function addrs()
  setup(pin)
  tbl = {}
  ow.reset_search(pin)
  repeat
    addr = ow.search(pin)
    if(addr ~= nil) then
      table.insert(tbl, addr)
    end
    tmr.wdclr()
  until (addr == nil)
  ow.reset_search(pin)
  return tbl
end

function readNumber(addr, unit)
  result = nil
  setup(pin)
  flag = false
  if(addr == nil) then
    ow.reset_search(pin)
    count = 0
    repeat
      count = count + 1
      addr = ow.search(pin)
      tmr.wdclr()
    until((addr ~= nil) or (count > 100))
    ow.reset_search(pin)
  end
  if(addr == nil) then
    return result
  end
  crc = ow.crc8(string.sub(addr,1,7))
  if (crc == addr:byte(8)) then
    if ((addr:byte(1) == 0x10) or (addr:byte(1) == 0x28)) then
      -- print("Device is a DS18S20 family device.")
      ow.reset(pin)
      ow.select(pin, addr)
      ow.write(pin, 0x44, 1)
      -- tmr.delay(1000000)
      present = ow.reset(pin)
      ow.select(pin, addr)
      ow.write(pin,0xBE,1)
      --print("P="..present)
      data = nil
      data = string.char(ow.read(pin))
      for i = 1, 8 do
        data = data .. string.char(ow.read(pin))
      end
      crc = ow.crc8(string.sub(data,1,8))
      if (crc == data:byte(9)) then
        t = (data:byte(1) + data:byte(2) * 256)
        return t
      end
      tmr.wdclr()
    end
  end
  return result
end

function read(addr, unit)
  t = readNumber(addr, unit)
  if (t == nil) then
    return nil
  else
    return t
  end
end

-- Return module table
return M