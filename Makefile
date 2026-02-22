.PHONY: bridge-up bridge-down test-sitl test-sitl-strict

SITL_NAME := ardupilot-sitl
SITL_IMAGE := radarku/ardupilot-sitl:eff32c1f98152ac3d1dc09a1e475733b73ce569f
MAVPROXY_LOG := /tmp/mavkit-mavproxy.log
MAVPROXY_PID := /tmp/mavkit-mavproxy.pid
MAVKIT_SITL_UDP_BIND ?= 0.0.0.0:14550

bridge-up:
	docker rm -f $(SITL_NAME) >/dev/null 2>&1 || true
	docker pull $(SITL_IMAGE)
	docker run -d --rm --name $(SITL_NAME) -p 5760:5760 \
		--entrypoint /ardupilot/build/sitl/bin/arducopter \
		$(SITL_IMAGE) \
		--model + --speedup 1 --defaults /ardupilot/Tools/autotest/default_params/copter.parm \
		--home 42.3898,-71.1476,14.0,270.0 -w
	python -c 'import socket,time,sys; deadline=time.time()+120; ok=False;\
while time.time()<deadline:\
 s=socket.socket(socket.AF_INET,socket.SOCK_STREAM); s.settimeout(1);\
 ok2=False;\
 try:\
  s.connect(("127.0.0.1",5760)); ok2=True\
 except OSError:\
  time.sleep(1)\
 finally:\
  s.close();\
 if ok2:\
  ok=True; break\
print("SITL TCP ready" if ok else "Timed out waiting for SITL TCP"); sys.exit(0 if ok else 1)'
	nohup uvx --from mavproxy --with future --python 3.11 mavproxy.py \
		--master=tcp:127.0.0.1:5760 \
		--out=udp:127.0.0.1:14550 \
		--daemon --non-interactive \
		--default-modules=link,signing,log,wp,rally,fence,param,relay,tuneopt,arm,mode,calibration,rc,auxopt,misc,cmdlong,battery,terrain,output,layout \
		>$(MAVPROXY_LOG) 2>&1 < /dev/null &
	printf "%s" $$! > $(MAVPROXY_PID)
	python -c 'import socket,time,sys; sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); sock.bind(("0.0.0.0",14550)); sock.settimeout(1); deadline=time.time()+180; ok=False;\
while time.time()<deadline:\
 try:\
  data,_=sock.recvfrom(4096);\
  if data:\
   ok=True; break\
 except TimeoutError:\
  pass\
sock.close(); print("UDP telemetry ready" if ok else "Timed out waiting for UDP telemetry"); sys.exit(0 if ok else 1)'

test-sitl:
	MAVKIT_SITL_UDP_BIND=$(MAVKIT_SITL_UDP_BIND) cargo test --test sitl_roundtrip -- --ignored --nocapture --test-threads=1

test-sitl-strict:
	MAVKIT_SITL_UDP_BIND=$(MAVKIT_SITL_UDP_BIND) MAVKIT_SITL_STRICT=1 cargo test --test sitl_roundtrip -- --ignored --nocapture --test-threads=1

bridge-down:
	if [ -f $(MAVPROXY_PID) ]; then kill "$$(cat $(MAVPROXY_PID))" >/dev/null 2>&1 || true; fi
	pkill -f "[m]avproxy.py --master=tcp:127.0.0.1:5760" >/dev/null 2>&1 || true
	docker rm -f $(SITL_NAME) >/dev/null 2>&1 || true
	rm -f $(MAVPROXY_PID)
