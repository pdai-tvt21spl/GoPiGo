import time
from easygopigo3 import EasyGoPiGo3 # importing the EasyGoPiGo3 class
#from line_follower import line_sensor
from di_sensors.easy_line_follower import EasyLineFollower
#static values for car instance could be provided as commandline arguments
ID = 0				#id of car exposed to mqtt
maxSpeed = 350			#the speed
envBlack = [0.045, 0.045, 0.045, 0.045, 0.045]
envWhite = [0.13, 0.13, 0.13, 0.13, 0.13]
lineBlackTrigger = [(envWhite[0]+envBlack[0])/2, (envWhite[1]+envBlack[1])/2, (envWhite[2]+envBlack[2])/2, (envWhite[3]+envBlack[3])/2, (envWhite[4]+envBlack[4])/2]    #light cutoff values


errorMaskLine   = [70.0, 30.0, 0.0, -30.0, -70.0]	#normal line

#pid values
proportionGain  = 0.90
integralGain    = 0.05
integralMax	= 0.10
derivitiveGain  = 0.50


def clamp(value, min, max):
	if value < min:
		return min;
	if value > max:
		return max;
	return value;


class movementContext:
	def __init__(self, maxSpeed, state, lineBlackTrigger, gpg, propGain, intgGain, intgMax, deriGain):
 		self.maxSpeed = maxSpeed
 		self.state = state  #"waitBridge", "idle", "error", "line", "waitLoad", "waitUnload"
 		self.lineBlackTrigger = lineBlackTrigger
# 		self.Proportional = 0
 #		self.Integral = 0
 #		self.Derivitive = 0
 		self.gpg = gpg
 		#pid values
 		self.Pg = propGain
 		self.Ig = intgGain
 		self.It = 0
 		self.Im = intgMax
 		self.Dg = deriGain
 		self.Ep = 0


class carContext:
	def __init__(self, id, maxSpeed, lineBlackTrigger, gpg):
		self.id = id
		self.moveCtx = movementContext(maxSpeed, "idle", lineBlackTrigger, gpg, proportionGain, integralGain, integralMax, derivitiveGain)



def getLightPattern(line):
	val=[0, 0, 0, 0, 0]
	for i in range(5):
		valn = line.read("raw")
		for ii in range(5):
			val[ii] += valn[ii]
	for i in range(len(val)):
#		val[i] = val[i]/(1-(val[i]/6)) # 6;
		val[i] /= 5 #val[i]-1
#		val[i] = val[i]
	return val


#0 = line
#1 = package load/unloadpoint
#2 = bridge point
#3 = no line
def interpretLightPattern(lightPattern, carContext, line):
	clights = [0, 0, 0, 0, 0]
	for i in range(len(lightPattern)):
		clights[i] = 1 if lightPattern[i] < carContext.moveCtx.lineBlackTrigger[i] else 0;
	print(clights)
	#check for [1, 0, 1, 0, 1] pattern and inform mqtt of either unload or load procedure
	if clights == [1, 0, 1, 0, 1]:
		carContext.moveCtx.gpg.stop()
		return 1
	#check for [1, 1, 1, 0, 0] or [0, 0, 1, 1, 1] turn pattern
	if clights == [1, 1, 1, 0, 0]:
		carContext.moveCtx.gpg.drive_cm(5.8, True)
		carContext.moveCtx.gpg.turn_degrees(-90)
		getLightPattern(line)
		return 0
	if clights == [0, 0, 1, 1, 1]:
		carContext.moveCtx.gpg.drive_cm(5.8, True)
		carContext.moveCtx.gpg.turn_degrees(90)
		getLightPattern(line)
		return 0
	#check for [1, 0, 0, 0, 1] bridge pattern
	if clights == [1, 0, 0, 0, 1]:
		carContext.moveCtx.gpg.stop()
		return 2;

	if clights == [0, 0, 0, 0, 0]:
#		carContext.moveCtx.gpg.stop()
		carContext.moveCtx.gpg.set_speed(10)
#       return 3;
	#just continue forward
	return 0

def motorCtrl(lightPattern, ctx, ctime, ptime):
	#hard intager values
	hardErr = 0;
	for i in range(len(lightPattern)):
		hardErr += (1 if lightPattern[i] < ctx.moveCtx.lineBlackTrigger[i] else 0)*errorMaskLine[i];
	hardErr /= 100.0;

	#softer floating point values
	softErr = 0
	for i in range(len(lightPattern)):
		softErr += clamp(float(lightPattern[i]-envWhite[i])/float(envBlack[i]-envWhite[i]), 1, 0)*errorMaskLine[i];
	softErr = (softErr/100.0+hardErr*2)/3


	#set speed of movement
	ctx.moveCtx.gpg.set_speed(300-abs(225*softErr))
#	ctx.moveCtx.gpg.set_speed(300)

	#calculate PID
	prop = softErr;
	intg = clamp((softErr+ctx.moveCtx.It)/(ctime-ptime), ctx.moveCtx.Im*-1, ctx.moveCtx.Im)
	deri = clamp((softErr-ctx.moveCtx.Ep)/(ctime-ptime), -1, 1);
	print(prop*ctx.moveCtx.Pg, intg*ctx.moveCtx.Ig, deri*ctx.moveCtx.Dg);
	#final error and update pid internals for next loop
	controllError = clamp(prop*ctx.moveCtx.Pg + intg*ctx.moveCtx.Ig + deri*ctx.moveCtx.Dg, -1, 1);
	ctx.moveCtx.It = intg;
	ctx.moveCtx.Ep = softErr;

	#calculate motor controll values
	lMotor = 0;
	rMotor = 0;
	if hardErr == 0.0:
		lMotor = 100.0;
		rMotor = 100.0;
	elif hardErr > 0.0:
		rMotor = 100.0;
		lMotor = (1.0-controllError)*100.0
	else:
		lMotor = 100.0;
		rMotor = (-1.0-controllError)*-100.0
#	print(rMotor, "\t\t", lMotor)
#	ctx.moveCtx.gpg.steer(((lMotor-5)*2+7)*0.95, ((rMotor-5)*2+7)*0.95)
	ctx.moveCtx.gpg.steer(lMotor, rMotor)



def main():
	gpg = EasyGoPiGo3() # instantiating a EasyGoPiGo3 object
	button = gpg.init_button_sensor()
	distance = gpg.init_distance_sensor()
	line = EasyLineFollower()
	ctx = carContext(ID, maxSpeed, lineBlackTrigger, gpg);
	ctx.moveCtx.gpg.stop();

#	calibration helper
	for x in range(5):
		lits = getLightPattern(line)
		print(lits, interpretLightPattern(lits, ctx, line));
		time.sleep(1)

	#linefollow code
	ptime = time.time()
	lits = getLightPattern(line)

	while button.is_button_pressed() == False:
#		time.sleep(0.1)
		if distance.read() < 10:
			ctx.moveCtx.gpg.stop()
			print("roadBlock")
			continue;
#		if distance.read_mm() > 500:
#			print("hole")
#			continue;
#		time.sleep(1)
#	while interpretLightPattern(lits, ctx, line) == 0:
		if interpretLightPattern(lits, ctx, line) != 0:
			break;
		ctime = time.time();
		lits=getLightPattern(line)
		motorCtrl(lits, ctx, ctime, ptime)
		if button.is_button_pressed():
			break
		ptime = ctime
#		time.sleep(1)
	ctx.moveCtx.gpg.stop()
main()
