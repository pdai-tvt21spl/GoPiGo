import time
from easygopigo3 import EasyGoPiGo3
from di_sensors.easy_line_follower import EasyLineFollower
#static values for car instance could be provided as commandline arguments
ID = 0				#id of car exposed to mqtt
maxSpeed = 250
envBlack = [0.2590420332355816, 0.3118279569892473, 0.3176930596285435, 0.3225806451612903, 0.21896383186705767]
envWhite = [0.3509286412512219, 0.39296187683284456, 0.40371456500488756, 0.39296187683284456, 0.2903225806451613]
lineBlackTrigger = [(a + b) / 2 for a,b in zip(envBlack, envWhite)]


errorMaskLine	= [70.0, 30.0, 0.0, -30.0, -70.0]	#normal line

#pid controller configurations https://en.wikipedia.org/wiki/Proportional–integral–derivative_controller
proportionGain	= 0.70
proportionMax	= 1.00
integralGain	= 0.05
integralMax	= 0.10
derivitiveGain	= 0.40
derivitiveMax	= 1.00


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


class carContext:
	def __init__(self, id, gpg):
		self.id = id
		self.moveCtx = movementContext(gpg, proportionGain, proportionMax, integralGain, integralMax, derivitiveGain, derivitiveMax)


#0 = line
#1 = package load/unloadpoint
#2 = bridge point
#3 = no line
def interpretLightPattern(lightPattern, carContext, line):
	clights = [0, 0, 0, 0, 0]
	for i in range(len(lightPattern)):
		clights[i] = 1 if lightPattern[i] < lineBlackTrigger[i] else 0;
	print(clights)
	#check for [1, 0, 1, 0, 1] pattern and inform mqtt of either unload or load procedure
	if clights == [1, 0, 1, 0, 1]:
		carContext.moveCtx.gpg.stop()
		return 1
	#check for [1, 1, 1, 0, 0] or [0, 0, 1, 1, 1] turn pattern
	if clights == [1, 1, 1, 0, 0]:
		carContext.moveCtx.gpg.stop()
		carContext.moveCtx.It = 0;
		carContext.moveCtx.gpg.drive_cm(5.8, True)
		carContext.moveCtx.gpg.turn_degrees(-90)
		return 0
	if clights == [0, 0, 1, 1, 1]:
		carContext.moveCtx.gpg.stop()
		carContext.moveCtx.It = 0
		carContext.moveCtx.gpg.drive_cm(5.8, True)
		carContext.moveCtx.gpg.turn_degrees(90)
		return 0
	#check for [1, 0, 0, 0, 1] bridge pattern
	if clights == [1, 0, 0, 0, 1]:
		carContext.moveCtx.gpg.stop()
		return 2;

	if clights == [0, 0, 0, 0, 0]:
		carContext.moveCtx.gpg.stop()
		carContext.moveCtx.gpg.set_speed(100)
		carContext.moveCtx.gpg.steer(carContext.moveCtx.Ml*-1, carContext.moveCtx.Mr*-1)
#		carContext.moveCtx.gpg.set_speed(75)
		#carContext.moveCtx.gpg.drive_cm(-15)
		time.sleep(3)
		carContext.moveCtx.gpg.stop()
#		carContext.moveCtx.gpg.set_speed(10)
		return 0;
	#just continue forward
	return 0

def motorCtrl(lightPattern, ctx, ctime, ptime):
	#hard intager values
	hardErr = 0;
	for i in range(len(lightPattern)):
		hardErr += (1 if lightPattern[i] < lineBlackTrigger[i] else 0)*errorMaskLine[i];
	hardErr /= 100.0;

	#softer floating point values
	softErr = 0
	for i in range(len(lightPattern)):
		softErr += clamp(float(lightPattern[i]-envWhite[i])/float(envBlack[i]-envWhite[i]), 0, 1)*errorMaskLine[i];
#	softErr = (softErr/100.0+hardErr*2)/3
#	softErr = ((softErr/100.0)*2+hardErr)/3
	softErr /= 100.0

	speedVar = 0;
	for i in range(len(lightPattern)):
		speedVar += clamp(float(lightPattern[i]-envWhite[i])/float(envBlack[i]-envWhite[i]), 0, 1)*abs(errorMaskLine[i]);
	speedVar /= 200;

	#set speed of movement
	ctx.moveCtx.gpg.set_speed(maxSpeed-abs(maxSpeed*0.7*speedVar))

	#calculate PID
	prop = clamp(softErr, ctx.moveCtx.Pm*-1, ctx.moveCtx.Pm);
	intg = clamp((softErr+ctx.moveCtx.It)/(ctime-ptime), ctx.moveCtx.Im*-1, ctx.moveCtx.Im)
	deri = clamp((softErr-ctx.moveCtx.Ep)/(ctime-ptime), ctx.moveCtx.Dm*-1, ctx.moveCtx.Dm);

    #final error and update pid internals for next loop
	controllError = clamp(prop*ctx.moveCtx.Pg + intg*ctx.moveCtx.Ig + deri*ctx.moveCtx.Dg, -1, 1);
	ctx.moveCtx.It = intg;
	ctx.moveCtx.Ep = softErr;

	#calculate motor controll values
	lMotor = 0;
	rMotor = 0;
	if abs(softErr) <= 0.08:
		rMotor = 100;
		lMotor = 100;
	elif softErr > 0.0:
		rMotor = 100.0;
		lMotor = (1.0-controllError)*100.0
	else:
		lMotor = 100.0;
		rMotor = (-1.0-controllError)*-100.0
	ctx.moveCtx.gpg.steer(lMotor, rMotor)
	ctx.moveCtx.Ml = lMotor;
	ctx.moveCtx.Mr = rMotor;



def main():
	gpg = EasyGoPiGo3()
	button = gpg.init_button_sensor()
	distance = gpg.init_distance_sensor()
	line = EasyLineFollower()
	ctx = carContext(ID, gpg);
	ctx.moveCtx.gpg.stop();

#	calibration helper
	for x in range(5):
		lits = line.read()
		print(lits, interpretLightPattern(lits, ctx, line));
		time.sleep(1)

	ptime = time.time()
	lits = line.read()

	while button.is_button_pressed() == False:
		if distance.read() < 10:
			ctx.moveCtx.gpg.stop()
			continue;
		if interpretLightPattern(lits, ctx, line) != 0:
			ctx.moveCtx.gpg.stop()
			break;
		ctime = time.time();
		lits=line.read()
		motorCtrl(lits, ctx, ctime, ptime)
		if button.is_button_pressed():
			break;
		ptime = ctime
	ctx.moveCtx.gpg.stop()
main()
