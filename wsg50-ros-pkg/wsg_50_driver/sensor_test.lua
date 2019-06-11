cmd.register(0xB3);

F_IDX = 0;
SENSORS = 1;

READ_CMD = 1;
DATA_REGISTER = 0x62;

for i=0,finger.count()-1 do
  t = finger.type(i);
  printf("Finger %d is a %s finger\n",i, t );
  if t == "generic" then
    
    F_IDX = i;
  end
end

finger.interface(F_IDX, "spi", 140625, 8, 0, 0);
sleep(100);


--[[
set_reg = finger.spi(F_IDX, {2,0x66});
printf("Set reg: ");
for i=1, #set_reg do
    printf("%d ", set_reg[i]);
end
printf("\n");
sleep(100);

get_reg = finger.spi(F_IDX, {3,0});
printf("Get reg: ");
for i=1, #get_reg do
    printf("%d ", get_reg[i]);
end
printf("\n");
sleep(100);
--]]

--[[
set_reg = finger.spi(F_IDX, {2,0x62});
printf("Set reg: ");
for i=1, #set_reg do
    printf("%d ", set_reg[i]);
end
printf("\n");
sleep(100);

get_reg = finger.spi(F_IDX, {3,0});
printf("Get reg: ");
for i=1, #get_reg do
    printf("%d ", get_reg[i]);
end
printf("\n");
sleep(100);

--]]

read_msg = {}
read_msg[1] = READ_CMD;
for i = 0, SENSORS-1 do
   read_msg[2+i] = i; 
end

function test_loop()
  ret_vals = {};
  while true do
     ret_vals = finger.spi(F_IDX, {READ_CMD});
     if ret_vals[1] == 42 then
         ret_vals = finger.spi(F_IDX, {1});
         break;
     else
          printf("Not ready, received: "); 
          for i=1, #ret_vals do
            printf("%d ", ret_vals[i]);
          end

          printf("\n");
         sleep(5);   
     end
  end

  for i=1, #ret_vals do
    printf("%d ", ret_vals[i]);
  end
  printf("\n");
  
  sleep(30);
        
end

function loop()

  id, payload = cmd.read();

  if id == 0xB3 and #payload == 1 then
    ret_vals = finger.spi(F_IDX, payload[1]);
    printf("...received");
    for i=1, #ret_vals do
      printf(" %d", ret_vals[i]);
    end
    printf("\nSent %d...", payload[1]);
    
    if cmd.online() then
      cmd.send(id, ret_vals);
    end
  end

end

while true do
    if not pcall(test_loop) then 
      print("Error occured");
      sleep(100);
    end
  --[[  
  if cmd.online() then
    if not pcall(loop) then 
      print("Error occured");
      sleep(100);
    end
  else
    sleep(100)
  end
  ]]--
end
