import time
from easygopigo3 import EasyGoPiGo3
from di_sensors.easy_line_follower import EasyLineFollower
import paho.mqtt.client as mqtt
from enum import IntEnum
import queue
import json
import math
#024589
class CarState(IntEnum):
	WAIT_LOAD = 0
	TO_BRIDGE_WLOAD = 1
	WAIT_FOR_BRIDGE_WLOAD = 2
	CROSS_BRIDGE_WLOAD = 3
	TO_UNLOAD = 4
	WAIT_FOR_UNLOAD = 5
	TO_BRIDGE_WOLOAD = 6
	WAIT_BRIDGE_WOLOAD = 7
	CROSS_BRIDGE_WOLOAD = 8
	GETTING_LOAD = 9


#static values for car instance could be provided as commandline arguments
ID = 0				#id of car exposed to mqtt
maxSpeed = 200
envBlack = [0.2590420332355816, 0.3118279569892473, 0.3176930596285435, 0.3225806451612903, 0.21896383186705767]
envWhite = [0.3509286412512219, 0.39296187683284456, 0.40371456500488756, 0.39296187683284456, 0.2903225806451613]
lineBlackTrigger = [(a + b) / 2 for a,b in zip(envBlack, envWhite)]


errorMaskLine	= [70.0, 30.0, 0.0, -30.0, -70.0]	#normal line

#pid controller configurations
proportionGain	= 0.65 #0.75
proportionMax	= 5.0  #5.00
integralGain	= 0.18 #0.18
integralMax	= 0.30 #0.50
derivitiveGain	= 0.18 #0.18
derivitiveMax	= 1.0 #5.00


def clamp(value, min, max):
	if value < min:
		return min;
	if value > max:
		return max;
	return value;


class movementContext:
	def __init__(self, gpg, propGain, propMax, intgGain, intgMax, deriGain, deriMax):
		self.gpg = gpg
 		#pid values
		self.Pg = propGain
		self.Pm = propMax
		self.Ig = intgGain
		self.It = 0
		self.Im = intgMax
		self.Dg = deriGain
		self.Dm = deriMax
		self.Ep = 0
		self.Mr = 0
		self.Ml = 0

class musicContext:
	def __init__(self, gpg):
		self.noteI = 0;
		self.ptime = time.time();
		self.buz = gpg.init_buzzer('AD2');
		self.buz.sound(0);
		self.buz.sound_off();

class carContext:
	def __init__(self, id, gpg, state, car_id):
		self.id = id
		self.moveCtx = movementContext(gpg, proportionGain, proportionMax, integralGain, integralMax, derivitiveGain, derivitiveMax)
		self.state = state
		self.resp_queue = queue.Queue()
		self.car_id = car_id
		self.dir = 1;
		self.musCtx = musicContext(gpg);

	def next_state(self):
		print("Old state %s" % self.state)
		self.state = CarState((self.state + 1) % 10)
		print("New state %s" % self.state)

#0 = line
#2 = package load/unloadpoint
#0 = no line
def interpretLightPattern(lightPattern, carContext, line):
	clights = [0, 0, 0, 0, 0]
	for i in range(len(lightPattern)):
		clights[i] = 1 if lightPattern[i] < lineBlackTrigger[i] else 0;
	#print(clights)
	if clights == [1, 1, 1, 1, 1]:
		carContext.moveCtx.gpg.stop()
		return 2;

	if clights == [0, 0, 0, 0, 0]:
		carContext.moveCtx.gpg.turn_degrees(8*((math.ceil(carContext.moveCtx.Ep)-0.5)*2));
		return 0;
	#just continue forward
	return 0

def speakerCtrl(ctx, ctime):
    # jäätelö auto
    noteF = [ 293,  494,  587,  494, 392,  293,  494,  587,  494, 392, 293, 523,  523, 440,  440, 392, 0];
	noteL = [0.25, 0.25, 0.25, 0.25, 0.5, 0.25, 0.25, 0.25, 0.25, 0.5, 0.5, 0.5, 0.25, 0.5, 0.25, 1.5, 5];
    # suuri ja mahtava neuvosto liitto  (matakat nuotit eivät toimi pitäis kasvattaa semitoneja)
#	noteF = [262, 262, 262, 523, 523, 523, 523, 523, 523, 622, 622, 311, 466, 466, 466, 932, 523, 523,
#		587, 587, 294, 392, 392, 392, 392, 523, 523, 466, 466, 466, 415, 415, 466, 466, 311, 311,
#		311, 311, 349, 349, 349, 392, 392, 415, 415, 415, 415, 415, 466, 466, 932, 523, 523, 587,
#		587, 622, 622, 698, 698, 698, 698, 698, 698, 784, 784, 784, 622, 622, 698, 698, 587, 587,
#		587, 294, 392, 392, 392, 392, 523, 523, 466, 466, 466, 415, 415, 466, 466, 311, 311, 311,
#		0.0	];
#	noteL = [0.5, 0.50, 0.50, 0.50, 1.00, 1.00, 1.00, 1.00, 1.0, 1.00, 1.00, 1.00, 0.50, 0.50, 0.50, 0.50, 0.25, 0.25,
#		1.00, 1.00, 1.00, 0.50, 0.50, 0.50, 0.50, 1.00, 1.0, 0.50, 0.50, 0.50, 0.25, 0.25, 1.00, 1.00, 0.50, 0.5,
#		0.50, 0.50, 1.50, 1.50, 1.50, 0.25, 0.25, 1.00, 1.0, 0.50, 0.50, 0.50, 0.25, 0.25, 0.25, 1.00, 1.00, 0.5,
#		0.50, 0.50, 0.50, 1.50, 1.50, 1.50, 1.50, 1.50, 1.5, 1.50, 1.50, 1.50, 0.25, 0.25, 1.00, 1.00, 0.50, 0.5,
#		0.50, 0.50, 0.50, 1.50, 1.50, 1.50, 0.25, 0.25, 1.0, 1.00, 1.00, 0.50, 0.50, 0.50, 0.50, 1.00, 1.00, 0.5,
#		10.0];
    # dixie
#   noteF = [392, 392, 261, 261, 261, 293, 329, 349, 392, 392, 392, 392, 0];
#   noteL = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 10];
    # daisy bell
#   noteF = [784, 659, 523, 392,  440,  494,  523, 440, 523, 392, 0];
#    noteL = [1.0, 1.0, 1.0, 1.0, 0.25, 0.25, 0.25, 0.5, 0.5, 2.0, 10];
    # toreador
#    noteF = [494,    0, 554, 494,  415,    0, 415,    0, 415,    0, 370,  415,  440, 415,   0, 440,    0, 370,  494, 415,   0,  329, 277,  370, 247, 0];
#    noteL = [0.5, 0.25, 0.5, 0.25, 0.5, 0.25, 0.5, 0.25, 0.5, 0.25, 0.25, 0.5, 0.25, 1.0, 0.5, 0.5, 0.25, 0.5, 0.25, 1.0, 0.5,  0.5, 0.5, 0.25, 1.0, 10];

	delay=0.01 #0.00
	attack=0.05
	attackS=0.5
	hold=0.02
	decay=0.01
	sustain=1
	release=0.6

	dtime = ctime-ctx.musCtx.ptime;
	if dtime > noteL[ctx.musCtx.noteI]*0.5:
		ctx.musCtx.noteI = (ctx.musCtx.noteI+1)%len(noteF);
		ctx.musCtx.ptime = ctime;
	noteI = ctx.musCtx.noteI;

	if dtime < delay:
		ctx.musCtx.buz.sound(0);
	elif dtime < (delay+attack):
		ctx.musCtx.buz.sound(((dtime-delay)/attack)*(attackS*noteF[noteI]));
	elif dtime < (delay+attack+hold):
		ctx.musCtx.buz.sound(noteF[noteI]*attackS);
	elif dtime < (delay+attack+hold+decay):
		lerpN = (dtime-delay-attack-hold)/decay;
		ctx.musCtx.buz.sound(((attackS*noteF[noteI])*(1-lerpN))+(noteF[noteI]*lerpN));
	elif dtime < (delay+attack+hold+decay+sustain):
		ctx.musCtx.buz.sound(noteF[ctx.musCtx.noteI]);
	elif dtime < (delay+attack+hold+decay+sustain+release):
		ctx.musCtx.buz.sound((1-((dtime-delay-attack-hold-decay-sustain)/release))*noteF[noteI]);
	else:
		ctx.musCtx.buz.sound(0);

def lightCtrl(ctx, ctime):
	ind = math.sin(ctime*6.28);
	rind = math.sin(ctime*6.28/65);
	gind = math.sin(ctime*6.28/130);
	bind = math.sin(ctime*6.28/195);
	ctx.moveCtx.gpg.set_eye_color((int(ind*255), int(gind*255), int(bind*255)));
	#ind = 1
	if ctx.state in [1, 3, 4, 6, 8, 9]:
		if abs(ctx.moveCtx.Ep) <= 0.05:
			ctx.moveCtx.gpg.blinker_off("left");
			ctx.moveCtx.gpg.blinker_off("right");
			ctx.moveCtx.gpg.close_eyes();
		elif ctx.moveCtx.Ep > 0:
			#left
			ctx.moveCtx.gpg.blinker_off("right");
			ctx.moveCtx.gpg.close_left_eye();
#			if ind > 0:
			ctx.moveCtx.gpg.blinker_on("left");
			ctx.moveCtx.gpg.open_right_eye();
#			else:
#				ctx.moveCtx.gpg.blinker_off("left");
#				ctx.moveCtx.gpg.close_left_eye();
		else:
			#right
			ctx.moveCtx.gpg.blinker_off("left");
			ctx.moveCtx.gpg.close_right_eye();
#			if ind > 0:
			ctx.moveCtx.gpg.blinker_on("right");
			ctx.moveCtx.gpg.open_left_eye();
#			else:
#				ctx.moveCtx.gpg.blinker_off("right");
#				ctx.moveCtx.gpg.close_right_eye();
#	elif ctx.state in [2, 7]:
#		if ind > 0:
#			ctx.moveCtx.gpg.blinker_on("left");
#			ctx.moveCtx.gpg.blinker_on("right");
#			ctx.moveCtx.gpg.open_eyes();
#		else:
#			ctx.moveCtx.gpg.blinker_off("left");
#			ctx.moveCtx.gpg.blinker_off("right");
#			ctx.moveCtx.gpg.close_eyes();
	else: #[5, 0]
		ctx.moveCtx.gpg.blinker_off("right");
		ctx.moveCtx.gpg.blinker_off("left");
		ctx.moveCtx.gpg.close_eyes();

def motorCtrl(lightPattern, ctx, ctime, ptime):
	#hard intager values
#	hardErr = 0;
#	for i in range(len(lightPattern)):
#		hardErr += (1 if lightPattern[i] < lineBlackTrigger[i] else 0)*errorMaskLine[i];
#	hardErr /= 100.0;

	#softer floating point values
	softErr = 0
	for i in range(len(lightPattern)):
		softErr += clamp(float(lightPattern[i]-envWhite[i])/float(envBlack[i]-envWhite[i]), 0, 1)*errorMaskLine[i];
	softErr /= 100.0

	speedVar = 0;
	for i in range(len(lightPattern)):
		speedVar += clamp(float(lightPattern[i]-envWhite[i])/float(envBlack[i]-envWhite[i]), 0, 1)*abs(errorMaskLine[i]);
	speedVar /= 200;

	#set speed of movement
	ctx.moveCtx.gpg.set_speed(maxSpeed-abs(maxSpeed*0.7*speedVar))
	#ctx.moveCtx.gpg.set_speed(maxSpeed);

	#calculate PID
	prop = clamp(softErr, ctx.moveCtx.Pm*-1, ctx.moveCtx.Pm);
	intg = clamp((softErr+ctx.moveCtx.It)/(ctime-ptime), ctx.moveCtx.Im*-1, ctx.moveCtx.Im)
	deri = clamp((softErr-ctx.moveCtx.Ep)/(ctime-ptime), ctx.moveCtx.Dm*-1, ctx.moveCtx.Dm);

	#final error and update pid internals for next loop
	controllError = prop*ctx.moveCtx.Pg + intg*ctx.moveCtx.Ig + deri*ctx.moveCtx.Dg;
	ctx.moveCtx.It = intg;
	ctx.moveCtx.Ep = softErr;

	#calculate motor controll values
	lMotor = 0;
	rMotor = 0;
	if softErr > 0.0:
		rMotor = 100.0;
		lMotor = (1.0-controllError)*140.0-40.0
	else:
		lMotor = 100.0;
		rMotor = (-1.0-controllError)*-140.0-40.0
	ctx.moveCtx.gpg.steer(lMotor, rMotor)
	ctx.moveCtx.Ml = lMotor;
	ctx.moveCtx.Mr = rMotor;
#0257 49
#states
	#0 wait for load
	#1 goint to bridge with load
	#2 waiting for bridge with load
	#3 crossing the bridge with load
	#4 going to unload
	#5 waiting to unload
	#6 going to bridge w/o loadd
	#7 waiting bridge w/o load
	#8 crossing bridge w/o load
	#9 going to get load

def on_connect(client, userdata, flags, rc):
	if rc != 0:
		print("Failed to connect to server ("+rc+")")
		return

	client.subscribe("robot/gopigo/"+str(userdata.car_id)+"/ping")
	client.subscribe("robot/gopigo/"+str(userdata.car_id)+"/ext_update")
	client.subscribe("global/ping/request")

def on_message(client, userdata, msg):
	print(msg)
	if msg.topic.endswith("ping"):
		if len(msg.payload) > 0:
			return
		client.publish("robot/gopigo/"+str(userdata.car_id)+"/ping", json.dumps({ "id": str(userdata.car_id) }).encode("utf-8"))
	elif msg.topic.endswith("ext_update"):
		parsed = json.loads(msg.payload.decode("utf-8"))
		print("External update from: "+ parsed["from"])
		userdata.resp_queue.put(parsed["from"])
	elif msg.topic == "global/ping/request":
		if len(msg.payload) > 0:
			return
		client.publish("global/ping/response", json.dumps({ "id": str(userdata.car_id), "type": "gopigo" }).encode("utf-8"))
	else:
		print("Message on unknown topic: " + msg.topic)

def MQTTStateHandler(ctx, mqttclient):
	if ctx.state == CarState.WAIT_LOAD:
		mqttclient.publish("robot/dispenser/load", json.dumps({ "id": str(ctx.car_id) }))
		while ctx.resp_queue.get() != "dispenser":
			lightCtrl(ctx, time.time());
			time.sleep(1);
	elif ctx.state == CarState.WAIT_FOR_BRIDGE_WLOAD:
		mqttclient.publish("robot/bridge/move", json.dumps({ "position": "to_builder", "requestee": str(ctx.car_id) }))
		while ctx.resp_queue.get() != "bridge":
			lightCtrl(ctx, time.time());
			time.sleep(1);
	elif ctx.state in (CarState.TO_UNLOAD, CarState.GETTING_LOAD):
		mqttclient.publish("robot/bridge/unlock", json.dumps({ "id": str(ctx.car_id) }))
	elif ctx.state == CarState.WAIT_FOR_UNLOAD:
		mqttclient.publish("robot/final/unload", json.dumps({ "id": str(ctx.car_id) }))
		while ctx.resp_queue.get() != "final":
			lightCtrl(ctx, time.time());
			time.sleep(1);
	elif ctx.state == CarState.WAIT_BRIDGE_WOLOAD:
		mqttclient.publish("robot/bridge/move", json.dumps({ "position": "to_dispenser", "requestee": str(ctx.car_id) }))
		while ctx.resp_queue.get() != "bridge":
			lightCtrl(ctx, time.time());
			time.sleep(1);

def main():
	gpg = EasyGoPiGo3()
	button = gpg.init_button_sensor()
	distance = gpg.init_distance_sensor()
	line = EasyLineFollower()
	ctx = carContext(ID, gpg, CarState.GETTING_LOAD, ID);
	ctx.moveCtx.gpg.stop();

	client = mqtt.Client(userdata=ctx)
	client.on_connect = on_connect
	client.on_message = on_message

	ctx.musCtx.buz.sound_off();

	client.connect("192.168.1.130", 1883, 60)
	print(client.loop_start())
	ctx.moveCtx.gpg.drive_cm(10);
#	calibration helper
#	for x in range(5):
#		lits = line.read()
#		print(lits, interpretLightPattern(lits, ctx, line));
#		time.sleep(1)

	ptime = time.time()
	lits = line.read()
	ctx.musCtx.buz.sound_off();

	while True: # button.is_button_pressed() == False:
		ctime = time.time()
		lightCtrl(ctx, ctime)
		# someting to determen based of state
		nxtState = 0;
		if ctx.state in (0, 2, 5, 7): # if state is a wait state
			MQTTStateHandler(ctx, client)
			ctx.moveCtx.It =0;
			ctx.moveCtx.gpg.drive_cm(13.0);
			ctx.next_state()
			time.sleep(0.5)
		else:
            # run speaker code
			if ctx.state == 4: # in (4, 9):
				speakerCtrl(ctx, ctime);
            # stop on obstickle
			if distance.read() < 10:
				ctx.moveCtx.gpg.stop()
				continue;
            #check if we need to change state
            lits = line.read();
			nxtState = interpretLightPattern(lits, ctx, line)
			if nxtState != 0:
                #stop music and update lights
				ctx.moveCtx.gpg.stop();
				ctx.musCtx.buz.sound(0);
				ctx.musCtx.buz.sound_off();
				ctx.next_state()
				lightCtrl(ctx, ctime);
                #send mqtt data incase we are in a state that needs it
				if ctx.state in (4, 9):
					MQTTStateHandler(ctx, client)
					if ctx.state == 4:
						ctx.musCtx.noteI = 0;
						ctx.musCtx.ptime = ctime;
						ctx.musCtx.buz.sound_on()
						speakerCtrl(ctx, ctime);
					ctx.moveCtx.gpg.drive_cm(5)
				continue;
            #lits=line.read()
            #run motos controll code
			motorCtrl(lits, ctx, ctime, ptime)
			ptime = ctime;
            #check kill switch
            if button.is_button_pressed():
				break;
    #teardown
	ctx.moveCtx.gpg.stop()
	client.loop_stop(force=True)
	ctx.musCtx.buz.sound_off();
main()
