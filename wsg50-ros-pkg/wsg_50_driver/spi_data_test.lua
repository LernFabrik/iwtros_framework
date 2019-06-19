cmd.register(0xB3);

F_IDX = 0;

for i=0,finger.count()-1 do
  t = finger.type(i);
  printf("Finger %d is a %s finger\n",i, t );
  if t == "generic" then
    
    F_IDX = i;
    --[[
    printf("Power cycling finger %d...", i);
    finger.power(i, false);
    sleep(3000);
    finger.power(i, true);
    printf("power cycle complete\n");]]--
  end
end

finger.interface(F_IDX, "spi", 1000000, 8, 0, 0);

test_val = 0;

function test_loop()
  sleep(1);
  ret_vals = finger.spi(F_IDX, {0,test_val});
  printf("...received");
  for i=1, #ret_vals do
    printf(" %d", ret_vals[i]);
  end
  printf("\nSent %d...", test_val); 
  test_val = (test_val+1)%16;
  sleep(1000);
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
