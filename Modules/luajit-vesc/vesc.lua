#!/usr/bin/env luajit

local bxor = require'bit'.bxor
local lshift = require'bit'.lshift
local rshift = require'bit'.rshift
--
local coyield = require'coroutine'.yield
local floor = require'math'.floor
local tinsert = require'table'.insert

-- Universaal bit operation functions
local firstByte, firstTwoBytes
if pcall(require, 'bit') then
  local bit = require'bit'
  firstByte = function(v)
    return bit.band(v, 0xFF)
  end
  firstTwoBytes = function(v)
    return bit.band(v, 0xFFFF)
  end
  selectByte = function(v, ibyte)
    local nbits = (8 * (ibyte or 1))
    return firstByte(bit.rshift(val, nbits))
  end
else
  firstByte = function(v)
    return v & 0xFF
  end
  firstTwoBytes = function(v)
    return v & 0xFFFF
  end
  selectByte = function(v, ibyte)
    local nbits = 8 * (ibyte or 1)
    return firstByte(val >> nbits)
  end
end

-- Convenience functions
local function lshift16(val, amt)
  return firstTwoBytes(lshift(val, amt or 1))
end
local function rshift8(val, amt)
  return firstByte(rshift(val, amt or 1))
end

-- Implement CRC16-CCITT
local function generate_crc16_lut()
  local base = 0x1021
  local crc16_lut = {}
  for idx=0, 255 do
    local crc = 0
    local c = lshift16(idx, 8)
    for _=0, 7 do
      local ret = band(bxor(c, crc), 0x8000) ~= 0
      -- Update the CRC
      crc = ret and bxor(lshift16(crc), base) or lshift16(crc)
      c = lshift16(c)
    end
    tinsert(crc16_lut, crc)
  end
  return crc16_lut
end

local crc16_lut = generate_crc16_lut()

local lib = {
  RPM_PER_MPS = 5220,
  crc16_lut = crc16_lut
}

local function calculate_crc(payload, a, b)
  local sum = 0
  for i=(a or 1), (b or #payload) do
    local tmp = bxor(selectByte(sum), payload[i])
    local crc = crc16_lut[firstByte(tmp) + 1]
    sum = bxor(crc, lshift16(sum, 8))
  end
  return sum
end

local function set_crc(pkt)
  local len = #pkt
  local sum = calculate_crc(pkt, 3, len-3)
  pkt[len - 2] = rshift8(sum, 8)
  pkt[len - 1] = firstByte(sum)
  return pkt
end

local GET_FW_VERSION_ID = 0x00
function lib.firmware_version()
  return set_crc{
    0x02, 0x01,
    -- Set the payload
    GET_FW_VERSION_ID,
    -- No CRC, yet
    false, false,
    -- End the packet
    0x03
  }
end

local GET_SENSORS_ID = 0x04
function lib.sensors()
  return set_crc{
    0x02, 0x01,
    -- Set the payload
    GET_SENSORS_ID,
    -- No CRC, yet
    false, false,
    -- End the packet
    0x03
  }
end

local ID_SET_DUTY_CYCLE = 0x05
function lib.duty_cycle(val)
  -- Input range: -100.0, 100.0%
  -- Output range: -1e5, 1e5
  if type(val)~='number' then
    return false, "Bad duty cycle number!"
  elseif val<-100 or val>100 then
    return false, "Bad duty cycle range! [-100, 100]"
  end
  val = floor(val * 1e3)
  return set_crc{
    -- Set the length
    0x02, 0x05,
    -- Set the payload
    ID_SET_DUTY_CYCLE,
    rshift8(val, 24),
    rshift8(val, 16),
    rshift8(val, 8),
    firstByte(val),
    -- No CRC, yet
    false, false,
    -- End the packet
    0x03
  }
end

local MAX_CURRENT = 60000
local ID_SET_CURRENT = 0x06
function lib.current(val)
  -- input milliamps
  if type(val)~='number' then
    return false, "Bad current number!"
  elseif val<-MAX_CURRENT or val>MAX_CURRENT then
    return false, "Bad current range!"
  end
  val = floor(val)
  return set_crc{
    -- Set the length
    0x02, 0x05,
    -- Set the payload
    ID_SET_CURRENT,
    rshift8(val, 24),
    rshift8(val, 16),
    rshift8(val, 8),
    firstByte(val),
    -- No CRC, yet
    false, false,
    -- End the packet
    0x03
  }
end

local ID_SET_CURRENT_BRAKE = 0x07
function lib.current_brake(val)
  -- input milliamps
  if type(val)~='number' then
    return false, "Bad current number!"
  elseif val<-MAX_CURRENT or val>MAX_CURRENT then
    return false, "Bad current range!"
  end
  val = floor(val)
  return set_crc{
    -- Set the length
    0x02, 0x05,
    -- Set the payload
    ID_SET_CURRENT_BRAKE,
    rshift8(val, 24),
    rshift8(val, 16),
    rshift8(val, 8),
    firstByte(val),
    -- No CRC, yet
    false, false,
    -- End the packet
    0x03
  }
end

local MAX_RPM = 30000
local ID_SET_RPM = 0x08
function lib.rpm(val)
  -- input rotations per minute
  if type(val)~='number' then
    return false, "Bad rpm number!"
  elseif val<-MAX_RPM or val>MAX_RPM then
    return false, "Bad rpm range!"
  end
  val = floor(val)
  return set_crc{
    -- Set the length
    0x02, 0x05,
    -- Set the payload
    ID_SET_RPM,
    rshift8(val, 24),
    rshift8(val, 16),
    rshift8(val, 8),
    firstByte(val),
    -- No CRC, yet
    false, false,
    -- End the packet
    0x03
  }
end

local MAX_POSITION = 100000
local ID_SET_POSITION = 0x09
function lib.position(val)
  -- input motor position
  if type(val)~='number' then
    return false, "Bad position number!"
  elseif val<-MAX_POSITION or val>MAX_POSITION then
    return false, "Bad position range!"
  end
  val = floor(val)
  return set_crc{
    -- Set the length
    0x02, 0x05,
    -- Set the payload
    ID_SET_POSITION,
    rshift8(val, 24),
    rshift8(val, 16),
    rshift8(val, 8),
    firstByte(val),
    -- No CRC, yet
    false, false,
    -- End the packet
    0x03
  }
end

-- NOTE: Check this on later VESC revisions (v3.35)
local ID_SET_SERVO_POSITION = 11
function lib.servo_position(val)
  -- input a between 0 and 1
  if type(val)~='number' then
    return false, "Bad servo position number!"
  elseif val<0 or val>1 then
    return false, "Bad servo position range [0,1]!"
  end
  val = floor(val * 1e3)
  return set_crc{
    -- Set the length
    0x02, 0x03,
    -- Set the payload
    ID_SET_SERVO_POSITION,
    rshift8(val, 8),
    firstByte(val),
    -- No CRC, yet
    false, false,
    -- End the packet
    0x03
  }
end

-- Input payload table
local function parse_values(p, tbl)
  local id = p[1]
  if id == GET_FW_VERSION_ID then
    return p[2] * 16 + p[3]
  end
  if id ~= GET_SENSORS_ID or #p ~= 56 then
    return false, "Not a values packet: "..tostring(p[1])
  end
  if type(tbl)~='table' then tbl = {} end
  -- MOSFET Temperature in Celsius
  tbl.mos_C = {
    (lshift(p[2], 8) + p[3]) / 10,
    (lshift(p[4], 8) + p[5]) / 10,
    (lshift(p[6], 8) + p[7]) / 10,
    (lshift(p[8], 8) + p[9]) / 10,
    (lshift(p[10], 8) + p[11]) / 10,
    (lshift(p[12], 8) + p[13]) / 10,
  }
  -- Board Temperature in Celsius
  tbl.pcb_C = (lshift(p[14], 8) + p[15]) / 10
  -- current_motor:
  tbl.motor_mA = (lshift(p[16], 24) + lshift(p[17], 16)
        + lshift(p[18], 8) + p[19]) * 10
  -- current_in:
  tbl.battery_mA = lshift(p[20], 24) + lshift(p[21], 16)
    + lshift(p[22], 8) + p[23]
  -- PWM duty cycle, now
  tbl.pwm = (lshift(p[24], 8) + p[25]) / 10
  tbl.rpm = lshift(p[26], 24) + lshift(p[27], 16)
    + lshift(p[28], 8) + p[29]
  tbl.v_in = (lshift(p[30], 8) + p[31]) / 10
  -- Drawn capacity, in milliAmp-hours:
  tbl.drawn_mAh = (lshift(p[32], 24) + lshift(p[33], 16)
    + lshift(p[34], 8) + p[35]) / 10
  -- Charged capacity, in milliAmp-hours:
  tbl.charged_mAh = (lshift(p[36], 24) + lshift(p[37], 16)
    + lshift(p[38], 8) + p[39]) / 10
  -- Drawn energy, in milliWatt-hours:
  tbl.drawn_mWh = (lshift(p[40], 24) + lshift(p[41], 16)
        + lshift(p[42], 8) + p[43]) / 10
  -- Charged energy, in milliWatt-hours:
  tbl.charged_mWh = (lshift(p[44], 24) + lshift(p[45], 16)
        + lshift(p[46], 8) + p[47]) / 10
  -- Tachometer
  tbl.tach = lshift(p[48], 24) + lshift(p[49], 16)
    + lshift(p[50], 8) + p[51]
  tbl.tach_abs = lshift(p[52], 24) + lshift(p[53], 16)
    + lshift(p[54], 8) + p[55]
  -- MC Fault Code
  tbl.fault = p[56]
  return tbl
end
lib.parse_values = parse_values

-- Enumerate the parts of the packet
local VESC_START = 0
local VESC_LENGTH_LOW = 1
local VESC_LENGTH_HIGH = 2
local VESC_ID = 3
local VESC_DATA = 4
local VESC_CHECKSUM_HIGH = 5
local VESC_CHECKSUM_LOW = 6
local VESC_END = 7
local function update(str, out, pkt)
  -- Check if we have stateful information
  if type(pkt)~='table' then
    pkt = {state=VESC_START, buffer = ''}
  end
  -- Access a previous buffer
  local buffer = pkt.buffer or ''
  -- Append new data to our buffer
  if type(str)=='string' then
    buffer = buffer..str
  end
  for cursor=1, #buffer do
    local byte = buffer:byte(cursor)
    local err
    if pkt.state==VESC_LENGTH_LOW then
      pkt.len = pkt.len + byte
      if pkt.len==0 then
        err = "Zero length packet"
        -- pkt.state = VESC_START
      end
      pkt.state = VESC_ID
    elseif pkt.state==VESC_LENGTH_HIGH then
      pkt.len = lshift(byte, 8)
      pkt.state = VESC_LENGTH_LOW
    elseif pkt.state==VESC_ID then
      if byte ~= GET_SENSORS_ID and byte ~= GET_FW_VERSION_ID then
        err = string.format("Bad ID %d", byte)
        -- pkt.state = VESC_START
      end
      -- Packet ID is first part of the payload
      tinsert(pkt, byte)
      pkt.state = VESC_DATA
    elseif pkt.state==VESC_DATA then
      -- TODO: Just read in one chunk, without iterating the for loop through all bytes
      -- {buffer:byte(cursor, cursor+pkt.len-2)}
      -- NOTE: Should put right in the VESC_ID state ...
      tinsert(pkt, byte)
      if #pkt == pkt.len then
        pkt.state = VESC_CHECKSUM_HIGH
      elseif #pkt > pkt.len then
        err = "Large payload"
        pkt.state = VESC_START
      end
    elseif pkt.state==VESC_CHECKSUM_HIGH then
      pkt.crc = lshift(byte, 8)
      pkt.state = VESC_CHECKSUM_LOW
    elseif pkt.state==VESC_CHECKSUM_LOW then
      pkt.crc = pkt.crc + byte
      if pkt.crc ~= calculate_crc(pkt) then
        err = "Bad checksum"
      end
      pkt.state = VESC_END
    elseif pkt.state==VESC_END then
      if byte ~= 0x03 then
        err = "Bad stop byte"
      end
      out, err = parse_values(pkt, out)
      pkt.state = VESC_START
      if out then
        pkt.buffer = buffer:sub(cursor+1)
        return out, pkt
      end
    else -- elseif pkt.state==VESC_START then
      -- Default to start state
      if byte == 0x02 then
        pkt.state = VESC_LENGTH_LOW
      elseif byte==0x03 then
        pkt.state = VESC_LENGTH_HIGH
      else
        err = string.format("Bad start packet [0x%02X]\n", byte)
        pkt.state = VESC_START
      end
    end -- if/else
    -- TODO: Check the return bits here
    if err then
      pkt.buffer = buffer:sub(cursor+1)
      return false, pkt, err
    end
  end
  -- pkt keeps the statefulness
  -- Iterated through the whole buffer, so buffer is cleared
  pkt.buffer = ''
  return out, pkt
end

function lib.co_update(str)
  local state
  while true do
    state = update(str, state)
    str = coyield(state.out)
  end
end

function lib.update(new_data)
  local str = type(new_data)=='string' and new_data or ''
  local pkt_state = VESC_START
  local pkt_len = 0
  local pkt_crc = 0
  local pkt_payload = {}
  local pkt_done = false
  while true do
    local cursor = 1
    while cursor <= #str do
      local byte = str:byte(cursor)
      -- io.stderr:write(string.format("%d: %02X\n", pkt_state, byte))
      -- Keep yielding the packet ID
      -- to indicate the data we are receiving
      if pkt_state==VESC_START then
        if byte == 0x02 then
          pkt_state = VESC_LENGTH_LOW
        elseif byte==0x03 then
          pkt_state = VESC_LENGTH_HIGH
        else
          io.stderr:write(string.format(
            "Bad start packet [0x%02X]\n", byte))
          pkt_state = VESC_START
        end
        pkt_payload = {}
      elseif pkt_state==VESC_LENGTH_LOW then
        pkt_len = pkt_len + byte
        if pkt_len==0 then
          io.stderr:write"Zero length packet\n"
          -- pkt_state = VESC_START
        end
        pkt_state = VESC_ID
      elseif pkt_state==VESC_LENGTH_HIGH then
        pkt_len = lshift(byte, 8)
        pkt_state = VESC_LENGTH_LOW
      elseif pkt_state==VESC_ID then
        if byte ~= GET_SENSORS_ID and byte ~= GET_FW_VERSION_ID then
          io.stderr:write("Bad ID", byte, "\n")
          -- pkt_state = VESC_START
        end
        -- Packet ID is first part of the payload
        tinsert(pkt_payload, byte)
        pkt_state = VESC_DATA
      elseif pkt_state==VESC_DATA then
        tinsert(pkt_payload, byte)
        if #pkt_payload == pkt_len then
          pkt_state = VESC_CHECKSUM_HIGH
        elseif #pkt_payload > pkt_len then
          io.stderr:write"Large payload?\n"
          pkt_state = VESC_START
        end
      elseif pkt_state==VESC_CHECKSUM_HIGH then
        pkt_crc = lshift(byte, 8)
        pkt_state = VESC_CHECKSUM_LOW
      elseif pkt_state==VESC_CHECKSUM_LOW then
        pkt_crc = pkt_crc + byte
        pkt_state = VESC_END
      elseif pkt_state==VESC_END then
        pkt_state = VESC_START
        if byte ~= 0x03 then
          io.stderr:write"Bad stop byte\n"
        elseif pkt_crc ~= calculate_crc(pkt_payload) then
          io.stderr:write"Bad checksum\n"
        end
        pkt_done = true
        -- break
      end -- if/else
      cursor = cursor + 1
    end -- for
    -- Save the remainder as str
    str = str:sub(cursor)
    -- Ask for more data while returning the state
    if pkt_done then
      -- io.stderr:write("Done packet!!\n")
      local obj, msg = parse_values(pkt_payload)
      new_data = coyield(obj, obj and pkt_state or msg)
      pkt_len = 0
      pkt_crc = 0
      pkt_done = false
    else
      new_data = coyield(false, pkt_state)
    end
    -- Add any new data
    if type(new_data)=='string' then
      str = str..new_data
    end
  end -- while a string
end

return lib
