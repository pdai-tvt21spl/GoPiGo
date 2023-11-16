import time
from easygopigo3 import EasyGoPiGo3 # importing the EasyGoPiGo3 class
from line_follower import line_sensor

#static values for car instance could be provided as commandline arguments
ID = 0                      #id of car exposed to mqtt
maxSpeed = 10                   #the speed in cm/s
acceleration = 0.5              #how quickly to get to accelerate to max speed
envWhite = [880, 792, 804, 803, 856]
envBlack = [993, 972, 978, 980, 989]

lineBlackTrigger = [(envWhite[0]+envBlack[0])/2, (envWhite[1]+envBlack[1])/2, (envWhite[2]+envWhite[2])/2, (envWhite[3]+envBlack[3])/2, (envWhite[4]+envBlack[4])/2]    #light cutoff values


errorMaskLine   = [9.0, 4.0, 0.0, -4.0, -9.0]	#normal line

#pid values
proportionGain  = 0.50				#gain*error
integralGain    = 0.00				#gain*error*deltaTime
integralMax	= 0.00				#
derivitiveGain  = 0.00				#(gain*(error-prevError))/deltaTime


def clamp(value, min, max):
	if value < min:
		return min;
	if value > max:
		return max;
	return value;


class movementContext:
	def __init__(self, maxSpeed, acceleration, state, lineBlackTrigger, gpg, propGain, intgGain, intgMax, deriGain):
 		self.maxSpeed = maxSpeed
 		self.acceleration = acceleration
 		self.state = state  #"waitBridge", "idle", "error", "line", "waitLoad", "waitUnload"
 		self.lineBlackTrigger = lineBlackTrigger
 		self.Proportional = 0
 		self.Integral = 0
 		self.Derivitive = 0
 		self.gpg = gpg
 		#pid values
 		self.Pg = propGain
 		self.Ig = intgGain
 		self.It = 0
 		self.Im = intgMax
 		self.Dg = deriGain
 		self.Ep = 0


class carContext:
	def __init__(self, id, maxSpeed, acceleration, lineBlackTrigger, gpg):
		self.id = id
		self.moveCtx = movementContext(maxSpeed, acceleration, "idle", lineBlackTrigger, gpg, proportionGain, integralGain, integralMax, derivitiveGain)



def getLightPattern():
	val=line_sensor.read_sensor()
	val=line_sensor.read_sensor()
	return val


#0 = line
#1 = package load/unloadpoint
#2 = bridge point
#3 = no line
def interpretLightPattern(lightPattern, carContext):
	clights = [0, 0, 0, 0, 0]
	for i in range(len(lightPattern)):
		clights[i] = 0 if lightPattern[i] < carContext.moveCtx.lineBlackTrigger[i] else 1;
	print(clights)
	#check for [1, 0, 1, 0, 1] pattern and inform mqtt of either unload or load procedure
	if clights == [1, 0, 1, 0, 1]:
		carContext.moveCtx.gpg.stop()
		return 1
	#check for [1, 1, 1, 0, 0] or [0, 0, 1, 1, 1] turn pattern <- these might be useless if RL turning is smart enough
	if clights == [1, 1, 1, 0, 0]:
		carContext.moveCtx.gpg.turn_degrees(90)
		return 0
	elif clights == [0, 0, 1, 1, 1]:
		carContext.moveCtx.gpg.turn_degrees(-90)
		return 0
	#check for [1, 0, 0, 0, 1] bridge pattern
	if clights == [1, 0, 0, 0, 1]:
		carContext.moveCtx.gpg.stop()
		return 2;

	if clights == [0, 0, 0, 0, 0]:
		carContext.moveCtx.gpg.stop()
		return 3;
	#just continue forward
	return 0

def motorCtrl(lightPattern, ctx, ctime, ptime):
	clights = [0.0, 0.0, 0.0, 0.0, 0.0];
	errorC = 0;
	#hard intager values
#	for i in range(len(lightPattern)):
#		clights[i] = 0 if lightPattern[i] < ctx.moveCtx.lineBlackTrigger[i] else 1;
	#softer floating point values
	for i in range(len(lightPattern)):
		clights[i] = clamp(float(lightPattern[i]-envWhite[i])/float(envBlack[i]-envWhite[i]), 0, 1);
	#raw error calculation
	for i in range(len(clights)):
		errorC += errorMaskLine[i]*clights[i]
	errorC = errorC/13.0;
	print(clights, lightPattern, errorC)
	#ctx.moveCtx.gpg.set_speed(500-abs(500*errorC))
	ctx.moveCtx.gpg.set_speed(300)

#   #calculate PID
	prop = errorC;
	intg = min((errorC+ctx.moveCtx.It)/(ctime-ptime), ctx.moveCtx.Im); #increment integral to a maximum value
	deri = (errorC-ctx.moveCtx.Ep)/(ctime-ptime);
    #final error and update pid internals for next loop
	controllError = prop*ctx.moveCtx.Pg + intg*ctx.moveCtx.Ig + deri*ctx.moveCtx.Dg;
	ctx.moveCtx.It = intg;
	ctx.moveCtx.Ep = errorC;

	#calculate motor controll values
	lMotor = 0;
	rMotor = 0;
	if controllError == 0.0:
		lMotor = 100.0;
		rMotor = 100.0;
	elif controllError > 0.0:
		lMotor = 100.0-controllError*100.0;
		rMotor = 100.0-lMotor;
	else:
		rMotor = 100.0-controllError*-100.0;
		lMotor = 100.0-rMotor;

	print(rMotor, lMotor, controllError, errorC, clights)

#   lMotor = 0
#   rMotor = 0
#   lMul = [-10, 10, 20, 30, 40]
#   rMul = [40, 30, 20, 10, -10]
#   for i in range(5):
#       if lightPattern[i] > ctx.moveCtx.lineBlackTrigger[i]:
#           lMotor += lMul[i]
#           rMotot += rMul[i]
#    ctx.moveCtx.gpg.steer((rMotor-25)+10*2, (lMotor-25)+10*2)
	#send controllvalues to motors
	ctx.moveCtx.gpg.steer(lMotor, rMotor)



def main():
	gpg = EasyGoPiGo3() # instantiating a EasyGoPiGo3 object
	button = gpg.init_button_sensor()
	ctx = carContext(ID, maxSpeed, acceleration, lineBlackTrigger, gpg);
	ctx.moveCtx.gpg.stop();

	#calibration helper
	for x in range(5):
		lits = getLightPattern()
		print(lits, interpretLightPattern(lits, ctx));
		time.sleep(1)

	#linefollow code
	ptime = time.time()
	lits = getLightPattern()
	while interpretLightPattern(lits, ctx) == 0:
		ctime = time.time();
		motorCtrl(lits, ctx, ctime, ptime)
		lits=getLightPattern()
		if button.is_button_pressed():
			break
		ptime = ctime

	ctx.moveCtx.gpg.stop()
main()
