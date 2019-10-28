local signal = require('signal')

function ShutDownFN()
	print("shutdown")
	os.exit(1);
end

print(signal.SIGINT)

local ret = signal.signal(signal.SIGINT, ShutDownFN)
io.stderr:write("Ret: ", tostring(ret), "\n")
-- signal.signal(signal.SIGTERM, ShutDownFN)

-- signal:signal("SIGINT", ShutDownFN);

io.stderr:write("Loop...\n")

while (true) do
	print('annoying!')
end
