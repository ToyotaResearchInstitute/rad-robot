--Darwin OP Commmanager for NSL 2011
module(..., package.seeall);

--Added for Hardware config file
local cwd = unix.getcwd();
package.path = cwd.."/../Config/?.lua;"..package.path;

require('DynamixelPacket');
require('Dynamixel');
require('unix');
require('shm');
require('carray');
require('vector');
require('Config');
require('Transform')

print("Robot ID:",Config.game.robotID);

dirReverse = Config.servo.dirReverse;
posZero=Config.servo.posZero;
gyrZero=Config.gyro.zero;
legBias=Config.walk.servoBias;
idMap = Config.servo.idMap;
nJoint = #idMap;
scale={};
for i=1,nJoint do 
  scale[i]=Config.servo.steps[i]/Config.servo.moveRange[i];
end
for i=1,12 do 	
  posZero[i+5]=posZero[i+5]+legBias[i];
end

tLast=0;
count=1;
battery_warning=0;
chk_servo_no=0;
nButton = 0;

-- USB disconnect bug
USB_bug = 1;
USB_bug = 0;

for i = 1,#Config.servo.dirReverse do
   scale[Config.servo.dirReverse[i]] = -scale[Config.servo.dirReverse[i]];
end

-- Setup shared memory
function shm_init()
   shm.destroy('dcmSensor');
   sensorShm = shm.new('dcmSensor');
   sensorShm.time = vector.zeros(1);
   sensorShm.count = vector.zeros(1);
   sensorShm.position = vector.zeros(nJoint);
   sensorShm.button = vector.zeros(2); --OP has TWO buttons

   sensorShm.imuAngle = vector.zeros(3);
   sensorShm.imuAcc = vector.zeros(3);
   sensorShm.imuGyr = vector.zeros(3);
   sensorShm.imuAccRaw = vector.zeros(3);
   sensorShm.imuGyrRaw = vector.zeros(3);
   sensorShm.imuGyrBias=vector.zeros(3); --rate gyro bias
   sensorShm.temperature=vector.zeros(nJoint);
   sensorShm.battery=vector.zeros(nJoint);
   sensorShm.updatedCount =vector.zeros(1);   --Increases at every cycle

   shm.destroy('dcmActuator');
   actuatorShm = shm.new('dcmActuator');
   print(nJoint)
   actuatorShm.command = vector.zeros(nJoint);
   actuatorShm.velocity = vector.zeros(nJoint);
   actuatorShm.hardness = vector.zeros(nJoint);
   actuatorShm.offset = vector.zeros(nJoint);
   actuatorShm.led = vector.zeros(1);

   actuatorShm.torqueEnable = vector.zeros(1); --Global torque on.off
   actuatorShm.slope=vector.ones(nJoint)*32; --default compliance slope is 32
   actuatorShm.slopeChanged=vector.ones(1);  --set compliance once
   actuatorShm.velocityChanged=vector.zeros(1);
   actuatorShm.hardnessChanged=vector.zeros(1);
   actuatorShm.torqueEnableChanged=vector.zeros(1);

   actuatorShm.backled = vector.zeros(3);  --red blue green
   actuatorShm.eyeled = vector.zeros(3);   --RGB15 eye led
   actuatorShm.headled = vector.zeros(3);  --RGB15 head led
   actuatorShm.headledChanged = vector.zeros(1);

   --Dummy variable (for compatibility with nao)
   actuatorShm.ledFaceRight=vector.zeros(24);
   actuatorShm.ledFaceLeft=vector.zeros(24);
   actuatorShm.ledChest=vector.zeros(24);

   --New PID parameters variables
   --Default value is (32,0,0)
   actuatorShm.p_param=vector.ones(nJoint)*32; 
   actuatorShm.i_param=vector.ones(nJoint)*0; 
   actuatorShm.d_param=vector.ones(nJoint)*0; 

   --SJ: list of servo IDs to read
   --0: Head only 1: All servos 2: Head+Leg
   --readID: 1 for readable, 0 for non-readable
   actuatorShm.readType=vector.zeros(1);   
   actuatorShm.readID=vector.zeros(nJoint); -
end

-- Setup CArray mappings into shared memory
function carray_init()
   sensor = {};
   for k,v in sensorShm.next, sensorShm do
      sensor[k] = carray.cast(sensorShm:pointer(k));
   end

   actuator = {};
   for k,v in actuatorShm.next, actuatorShm do
      actuator[k] = carray.cast(actuatorShm:pointer(k));
   end
end


function sync_command()
   local addr = 30;
   local ids = {};
   local data = {};
   local n = 0;
   for i = 1,#idMap do
      if (actuator.hardness[i] > 0) then
	 n = n+1;
	 ids[n] = idMap[i];
         local word=0;
	 word = posZero[i] + scale[i]*
	      (actuator.command[i]+actuator.offset[i]);
	 data[n] = math.min(math.max(word, 0), Config.servo.steps[i]-1);
      else
	 actuator.command[i] = sensor.position[i];
      end
   end
   if (n > 0) then
      Dynamixel.sync_write_word(ids, addr, data);
   end
end

function sync_hardness()
   local addr=34; --hardness is working with RX28
   local ids = {};
   local data = {};
   local n = 0;
   for i = 1,#idMap do
	 n = n+1;
	 ids[n] = idMap[i];
	 data[n] = 1023*actuator.hardness[i];
   end
   if (n > 0) then
      Dynamixel.sync_write_word(ids, addr, data);
   end
end

function torque_enable()
   local addr = 24;
   local ids = {};
   local data = {};
   local n = 0;
   for i = 1,#idMap do
	n = n+1;
	ids[n] = idMap[i];
	data[n] = actuator.torqueEnable[1];
   end
   if (n > 0) then
      Dynamixel.sync_write_byte(ids, addr, data);
   end   
   print("Torque enable changed")
end

--Servo feedback param for servomotors
--Used to stiffen support foot during kicking

function sync_slope()
   if Config.servo.pid==0 then --Old firmware.. compliance slope
     --28,29: Compliance slope positive / negative
     local addr={28,29};
     local ids = {};
     local data = {};
     local n = 0;
     for i = 1,#idMap do
	 n = n+1;
	 ids[n] = idMap[i];
	 data[n] = actuator.slope[i];
     end
     Dynamixel.sync_write_byte(ids, addr[1], data);
     Dynamixel.sync_write_byte(ids, addr[2], data);
   else --New firmware: PID parameters
     -- P: 26, I: 27, D: 28
     local addr={26,27,28};
     local ids = {};
     local data_p = {};
     local data_i = {};
     local data_d = {};
     local n = 0;
     for i = 1,#idMap do
	 n = n+1;
	 ids[n] = idMap[i];
	 data_p[n] = actuator.p_param[i];
	 data_i[n] = actuator.i_param[i];
	 data_d[n] = actuator.d_param[i];
     end
     Dynamixel.sync_write_byte(ids, addr[1], data_p);
     Dynamixel.sync_write_byte(ids, addr[2], data_i);
     Dynamixel.sync_write_byte(ids, addr[3], data_d);
   end
end

function sync_battery()
--DarwinOP specific: only check leg servos
   chk_servo_no=(chk_servo_no%12)+1;
   sensor.battery[chk_servo_no+5]=Dynamixel.get_battery(chk_servo_no+5);
   sensor.temperature[chk_servo_no+5]=Dynamixel.get_temperature(chk_servo_no+5);

   local bat_min=200;
   for i=6,17 do
     if sensor.battery[i]>0 then
       bat_min=math.min(bat_min,sensor.battery[i]);
     end
   end
   if bat_min<Config.bat_low then battery_warning=1;
   else battery_warning=0;
   end
end

function sync_led()
--New function to turn on status LEDs
  local packet;

  if count%20==0 then --5 fps eye led refresh rate
    packet=actuator.eyeled[1]+32*actuator.eyeled[2]+1024*actuator.eyeled[3];
    Dynamixel.sync_write_word({200},28,{packet});
    unix.usleep(100);
  end

  if battery_warning==1 then
    packet=5;
    Dynamixel.sync_write_word({200},26,{packet});
    unix.usleep(100);
    battery_warning=0;
      
  elseif count%20==10 then --5 fps head led refresh rate
    packet=actuator.headled[1]+32*actuator.headled[2]+1024*actuator.headled[3];
    Dynamixel.sync_write_word({200},26,{packet});
    unix.usleep(100);
  end
	
  if count%400==225 then --0.25 fps back led refresh rate
    --Back LED	
   packet=actuator.backled[1]+2*actuator.backled[2]+4*actuator.backled[3];
   Dynamixel.sync_write_byte({200},25,{packet});
   unix.usleep(100);
  end
end

function nonsync_read()

  --Position reading
  local idToRead={1,2};   --Head only reading
    if actuator.readType[1]==1 then --All servo reading
      for i=1,#idMap do 
      idToRead[i]=i;
    end
  elseif actuator.readType[1]==3 then -- Read ankles only
    idToRead = {10,16}; --kankle ids
    for i = 1,#idMap do
      sensor.position[i] = actuator.command[i];
      end;
    else --if actuator.readType[1]==0 then --Head only reading
      for i = 3,#idMap do
      sensor.position[i] = actuator.command[i];
    end;

--[[
   elseif actuator.readType[1]==2 then --Head+Leg reading
     for i = 3,6 do
       sensor.position[i] = actuator.command[i];
     end;
     for i = 18,#idMap do
       sensor.position[i] = actuator.command[i];
     end;

     c_mod=count%4;
     if c_mod==1 then	idToRead={6,7,8};  
     elseif c_mod==2 then 	idToRead={9,10,11};  
     elseif c_mod==3 then	idToRead={12,13,14}
     else	idToRead={15,16,17};  
     end
   elseif actuator.readType[1]==3 then --No reading
     idToRead={}; 
     for i = 1,#idMap do
       sensor.position[i] = actuator.command[i];
     end;
--]]
  end
  -- Update the readings
  for i = 1,#idToRead do
    local id = idMap[idToRead[i]];
    --Sometimes DCM crashes here... maybe a bug
    local raw=null;
    if id then
      raw=Dynamixel.get_position(id);
      if raw then
        sensor.position[idToRead[i]] = (raw-posZero[i])/scale[i] - actuator.offset[i];
      end
    end
  end

  --IMU reading

  local data=Dynamixel.read_data(200,38,12);
  local offset=1;

  if data and #data>11 then
    for i=1,3 do
      sensor.imuGyr[Config.gyro.rpy[i]] =
        Config.gyro.sensitivity[i]*
        (DynamixelPacket.byte_to_word(data[offset],data[offset+1])-gyrZero[i]);

      sensor.imuAcc[Config.acc.xyz[i]] = 
        Config.acc.sensitivity[i]*
        (DynamixelPacket.byte_to_word(data[offset+6],data[offset+7])-Config.acc.zero[i]);

      sensor.imuGyrRaw[Config.gyro.rpy[i]]=DynamixelPacket.byte_to_word(data[offset],data[offset+1]);
      sensor.imuAccRaw[Config.acc.xyz[i]]=DynamixelPacket.byte_to_word(data[offset+6],data[offset+7]);
      offset = offset + 2;
    end
  end

  --Button reading
  data=Dynamixel.read_data(200,30,1);
  if data then
    sensor.button[1]=math.floor(data[1]/2);
    sensor.button[2]=data[1]%2;
  end
end

function entry()
  Dynamixel.open();
  --   Dynamixel.ping_probe();
  --We have to manually turn on the MC for OP   
  Dynamixel.set_torque_enable(200,1);
  unix.usleep(200000);
  -- Dynamixel.ping_probe();
  shm_init();
  carray_init();
  -- Read head and not legs
  actuator.readType[1]=1;
  -- Read only kankles
  actuator.readType[1]=3;
 
end

function update_imu()
  t=unix.time();
  if tLast==0 then tLast=t; end
  tPassed=t-tLast;
  tLast=t;
  iAngle=vector.new({sensor.imuAngle[1],sensor.imuAngle[2],sensor.imuAngle[3]});
  gyrAngleDelta = vector.new({sensor.imuGyr[1],sensor.imuGyr[2],sensor.imuGyr[3]})
                *math.pi/180 * tPassed; --dps to rps conversion

  --Angle transformation: yaw -> pitch -> roll
  local tTrans=Transform.rotZ(iAngle[3]);
  tTrans=tTrans*Transform.rotY(iAngle[2]);
  tTrans=tTrans*Transform.rotX(iAngle[1]);

  local tTransDelta=Transform.rotZ(gyrAngleDelta[3]);
  tTransDelta=tTransDelta*Transform.rotY(gyrAngleDelta[2]);
  tTransDelta=tTransDelta*Transform.rotX(gyrAngleDelta[1]);

  tTrans=tTrans*tTransDelta;
  iAngle=Transform.getRPY(tTrans);

  local accMag=sensor.imuAcc[1]^2+sensor.imuAcc[2]^2+sensor.imuAcc[3]^2;
  if accMag>Config.angle.gMin and accMag<Config.angle.gMax then
    local angY=math.atan2(sensor.imuAcc[1], math.sqrt(sensor.imuAcc[2]^2+sensor.imuAcc[3]^2) );
    local angX=math.atan2(sensor.imuAcc[2], math.sqrt(sensor.imuAcc[1]^2+sensor.imuAcc[3]^2) );
    iAngle[1], iAngle[2] =
      (1-Config.angle.accFactor)*iAngle[1]+Config.angle.accFactor*angX,
      (1-Config.angle.accFactor)*iAngle[2]+Config.angle.accFactor*angY;
  end

  sensor.imuAngle[1],sensor.imuAngle[2],sensor.imuAngle[3] = iAngle[1],iAngle[2],iAngle[3];
end

function update()

  -- Grab all tty's and see if different than
  -- the previous tty (Felix's USB bug)
  if( USB_bug == 1 ) then
    local ttyname = 'none'
    local ttys = unix.readdir("/dev");
    for i=1,#ttys do
      if (string.find(ttys[i], "tty.usb") or
        string.find(ttys[i], "ttyUSB")) then
        ttyname = "/dev/"..ttys[i];
        break;
      end 
    end
    if( ttyname ~= Dynamixel.dttyname ) then
      if( Dynamixel.dttyname ~= nil ) then
      print('bug detected! Found '..ttyname..', but connected original is: '.. Dynamixel.dttyname);
      -- Close, then wait .2 seconds, then try opening again
      Dynamixel.close();
      unix.sleep( 200000 ); -- .2 seconds
      Dynamixel.open();
      end
    end
  end

   nonsync_read();
   update_imu();

   count=count+1;
   sensor.updatedCount[1]=count%100; --This count indicates whether DCM has processed current reading or not
   if count%100==0 then 
	sync_battery();
   end
   if actuator.hardnessChanged[1]==1 then
	sync_hardness();
	unix.usleep(100);
	actuator.hardnessChanged[1]=0;
   end
   if actuator.slopeChanged[1]==1 then
	sync_slope();
	actuator.slopeChanged[1]=0;
        unix.usleep(100);
   end
   if actuator.velocityChanged[1]==1 then
        sync_velocity();
	actuator.velocityChanged[1]=0;
        unix.usleep(100);
   end
   if actuator.torqueEnableChanged[1]==1 then
        torque_enable();
	actuator.torqueEnableChanged[1]=0;
        unix.usleep(100);
   end

   sync_command();
   unix.usleep(100);

   sync_led();
end

function exit()
   Dynamixel.close();
end

