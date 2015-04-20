###############################################
# Post Processing R Script to compute
# the Allan variance over the STIM300 5g
# Javier Hidalgo Carrio
# DFKI-RIC ESTEC March 2014
###############################################

#libraries
library(allanvar)

# You may change this path
setwd ("/home/javi/flatfish/development/post-process_data/20150227-0019_static_8h_imu_run")

#load("stim300_acc_16bnw_500hz_analysis.Rdata")
#load("stim300_acc_16bnw_250hz_analysis.Rdata")
#load("stim300_acc_262bnw_500hz_analysis.Rdata")

#############
# TEST Acc
#############
#values <- read.table ("stim300_acc_16bnw_500hz.data", sep=" ")
#values <- read.table ("stim300_acc_16bnw_250hz.data", sep=" ")
#values <- read.table ("stim300_acc_262bnw_500hz.data", sep=" ")
#values <- read.table ("kvh_imu_acc.0.data", sep=" ")
values <- read.table ("stim300_imu_acc.0.data", sep=" ")

names(values) = c("time", "accx", "accy", "accz")

str(values)

#Time is in microseconds (10^-6)
delta <- diff(values$time)
delta <- mean(delta)/1000000

########################## X Axis ##########################
imu <- ts(as.numeric(array(na.omit(values$accx))), deltat=mean(delta))

#Frequency
frequency (imu)

#### Calculating the Allan Variance for the accelerometers ####
avacc1x <- avar (imu@.Data, frequency (imu))

########################## Y Axis ##########################
imu <- ts(as.numeric(array(na.omit(values$accy))), deltat=mean(delta))

#Frequency
frequency (imu)

#### Calculating the Allan Variance for the accelerometers ####
avacc1y <- avar (imu@.Data, frequency (imu))

########################## Z Axis ##########################
imu <- ts(as.numeric(array(na.omit(values$accz))), deltat=mean(delta))

#Frequency
frequency (imu)

#### Calculating the Allan Variance for the accelerometers ####
avacc1z <- avar (imu@.Data, frequency (imu))


#### Ploting the results ####
plotCI (x=avacc1x$time, y=sqrt(avacc1x$av), uiw =  sqrt(avacc1x$av)*avacc1x$error, xaxt="n", yaxt="n", pch=0, gap=0, col="red", slty=par("lty.2") , log= "xy", xlab="", ylab="")
lines (avacc1x$time,sqrt(avacc1x$av), col="red")
plotCI (avacc1y$time,sqrt(avacc1y$av),uiw =  sqrt(avacc1y$av)*avacc1y$error,log= "xy", xaxt="n", yaxt="n", pch=8, gap=0, col="green", xlab="", ylab="", add=TRUE)
lines (avacc1y$time,sqrt(avacc1y$av), col="green")
plotCI (avacc1z$time,sqrt(avacc1z$av),uiw =  sqrt(avacc1y$av)*avacc1y$error, log= "xy", xaxt="n", yaxt="n", pch=17, gap=0,  col="blue", xlab="", ylab="", add=TRUE)
lines (avacc1z$time,sqrt(avacc1z$av), col="blue")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
axis(2, c(0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m/s^2)")

legend(50, 5e-03, c("Accelerometer X", "Accelerometer Y", "Accelerometer Z"),  col = c("red", "green", "blue"), pch=c(0, 8, 17))

#### Ploting the results ####
plotCI (x=avacc1x$time, y=sqrt(avacc1x$av), uiw =  sqrt(avacc1x$av)*avacc1x$error, xaxt="n", yaxt="n", pch=0, gap=0, col="red", slty=par("lty.2") , log= "xy", xlab="", ylab="")
lines (avacc1x$time,sqrt(avacc1x$av), col="red")
points (avacc1y$time,sqrt(avacc1y$av), log= "xy", xaxt="n", yaxt="n", pch=8, col="green", xlab="", ylab="", add=TRUE)
lines (avacc1y$time,sqrt(avacc1y$av), col="green")
points (avacc1z$time,sqrt(avacc1z$av), log= "xy", xaxt="n", yaxt="n", pch=17, col="blue", xlab="", ylab="", add=TRUE)
lines (avacc1z$time,sqrt(avacc1z$av), col="blue")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
axis(2, c(0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000))
grid(equilogs=TRUE, lwd=1, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m/s^2)")

legend(50, 5e-03, c("Accelerometer X", "Accelerometer Y", "Accelerometer Z"),  col = c("red", "green", "blue"), pch=c(0, 8, 17))

#### Plotting the results in a file ####
png(filename = "stim300_acc_allanvar.png", width=1024, height=768, units = "px", pointsize = 22, bg = "white", res = NA)
plotCI (x=avacc1x$time, y=sqrt(avacc1x$av), uiw =  sqrt(avacc1x$av)*avacc1x$error, xaxt="n", yaxt="n", pch=0, gap=0, col="red", slty=par("lty.2") , log= "xy", xlab="", ylab="")
lines (avacc1x$time,sqrt(avacc1x$av), col="red")
plotCI (avacc1y$time,sqrt(avacc1y$av),uiw =  sqrt(avacc1y$av)*avacc1y$error,log= "xy", xaxt="n", yaxt="n", pch=8, gap=0, col="green", xlab="", ylab="", add=TRUE)
lines (avacc1y$time,sqrt(avacc1y$av), col="green")
plotCI (avacc1z$time,sqrt(avacc1z$av),uiw =  sqrt(avacc1y$av)*avacc1y$error, log= "xy", xaxt="n", yaxt="n", pch=17, gap=0,  col="blue", xlab="", ylab="", add=TRUE)
lines (avacc1z$time,sqrt(avacc1z$av), col="blue")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
axis(2, c(0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000))
grid(equilogs=TRUE, lwd=2.0, col="orange")
title(xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (m/s^2)", prob=TRUE,
     cex.lab=1.5, cex.axis=1.5, cex.main=1.5, cex.sub=1.5)

legend(50, 5e-03, c("Accelerometer X", "Accelerometer Y", "Accelerometer Z"),  col = c("red", "green", "blue"), pch=c(0, 8, 17))

dev.off()

### Delete all values
rm (values, imu)

#### Save the result in a R image
#save.image (file = "stim300_acc_16bnw_500hz_analysis.Rdata")
#save.image (file = "stim300_acc_16bnw_250hz_analysis.Rdata")
#save.image (file = "stim300_acc_262bnw_500hz_analysis.Rdata")

#######################################
#### Calculate the values (STIM300 Acc) #
#######################################
plotav (avacc1x)

##
#Random Walk, can be directly obtained by reading the slope line at T = 1
##
#Test 1 # sample 96
approx (x=c(avacc1x$time[7], avacc1x$time[8]), y= c(sqrt(avacc1x$av[7]), sqrt(avacc1x$av[8])), n=100)
0.0010324520 #KVH 1750 m/s^2/sqrt(Hz) or m/s/sqrt(s)
0.0009124428 #KVH 1750 m/s^2/sqrt(Hz) or m/s/sqrt(s)
0.0009805496 #STIM300 (5 hours test) 33Hz bandwidth 10g Dagon
0.001044618 #STIM300 (3 hours test) 66Hz bandwidth 10g Dagon
0.0009531720 #STIM300 (8 hours test) 33Hz bandwidth 10g Dagon

approx (x=c(avacc1y$time[7], avacc1y$time[8]), y= c(sqrt(avacc1y$av[7]), sqrt(avacc1y$av[8])), n=100)
0.001339378 #KVH 1750 m/s^2/sqrt(Hz)
0.0009831725 #KVH 1750 m/s^2/sqrt(Hz) or m/s/sqrt(s)
0.001094544 #STIM300 (5 hours test) 33Hz bandwidth 10g Dagon
0.001164101 #STIM300 (3 hours test) 66Hz bandwidth 10g Dagon
0.001048870 #STIM300 (8 hours test) 33Hz bandwidth 10g Dagon


approx (x=c(avacc1z$time[7], avacc1z$time[8]), y= c(sqrt(avacc1z$av[7]), sqrt(avacc1z$av[8])), n=100)
0.0009739990 #KVH 1750 m/s^2/sqrt(Hz)
0.0009442396 #KVH 1750 m/s^2/sqrt(Hz) or m/s/sqrt(s)
0.001024745 #STIM300 (5 hours test) 33Hz bandwidth 10g Dagon
0.001089919 #STIM300 10g
0.001022390 #STIM300 (8 hours test) 33Hz bandwidth 10g Dagon

#DATASHEET for KVH1750  ACC 10g
0.0011772 #m/s^2/sqrt(Hz) or or m/s/sqrt(s)
(0.0011772/0.0167) #m/s/sqrt(hr)

#DATASHEET for STIM300  ACC 10g
0.001169  #m/s^2/sqrt(Hz) or m/s/sqrt(s)
(0.001169/0.0167) #m/s/sqrt(hr)

#DATASHEET for STIM300  ACC 5g
0.000353 #m/s^2/sqrt(Hz) or m/s/sqrt(s)



##
#Bias Instability, can be directly obtained by reading the 0 slope line 
##
#Test 1 Bias instability Coeff
sqrt(avacc1x$av[14])/(sqrt(2*log(2)/pi))#KVH1750
0.000579399 #KVH1750 (8 hours test)
sqrt(avacc1x$av[14])/(sqrt(2*log(2)/pi))#STIM300
0.001156174 #STIM300 (5 hours test) 33Hz bandwidth 10g Dagon with filter on
0.0005563053 #STIM300 (8 hours test) 33Hz bandwidth 10g
0.0006094821 #(2 hours test 10g acc)

#COMPARISON WITH OTHERS
0.0003073389 #STIM300 16bnw_500hz(4 hours test 5g acc)
0.0005639343 # STIM300 (5 hours test) 10g stim300 acc coincident with datasheet value
0.0003110737 #STIM300 m/s^2 datasheet 0.0005 m/s^2
0.0001213269 #IMAR m/s^2 datasheet 0.01 m/s^2
0.0004976398 #XSens m/s^2

sqrt(avacc1y$av[14])/(sqrt(2*log(2)/pi))#KVH1750
0.0005860783 #KVH1750 (8 hours test)
sqrt(avacc1y$av[14])/(sqrt(2*log(2)/pi))
0.001200315 #STIM300 (5 hours test) 33Hz bandwidth 10g Dagon with filter on
0.0006042829 #STIM300 (8 hours test) 33Hz bandwidth 10g
0.0006335726 #(2 hours test 10g acc)

#COMPARISON WITH OTHERS
0.0002858801 #16bnw_500hz (4 hours test 5g acc)
0.0006412585 # (5 hours test) 10g stim300
0.0003967415 #STIM300 m/s^2
0.000117481 #iMAR m/s^2
0.0005772482 #XSens m/s^2

sqrt(avacc1z$av[14])/(sqrt(2*log(2)/pi))#KVH1750
0.0005393933 #KVH1750 (8 hours test)
sqrt(avacc1z$av[14])/(sqrt(2*log(2)/pi))
0.0008595971 #STIM300 (5 hours test) 33Hz bandwidth 10g Dagon with filter on
0.0006206795 #STIM300 (8 hours test) 33Hz bandwidth 10g
0.0006150693 #(2 hours test 10g acc)

#COMPARISON WITH OTHERS
0.0002847117 #16bnw_500hz (4 hours test 5g acc)
0.0005866994 # (5 hours test) 10g stim300 acc coincident with datasheet value
0.0003779767 #STIM300 m/s^2
0.0007197698 #iMAR m/s^2
0.0006744514 #XSens m/s^2

##
#Rate Random Walk can be obtained by reading the allan variance value
#by a slope at +1/2. K=sqrt(3)*allandeviation(t)/sqrt(t)
# or K=sqrt((3*allandeviation(t))/sqrt)t))
##
sqrt((3.0 * avacc1x$av[20])/avacc1x$time[20])#KVH1750 & STIM300
2.470794e-05 #KVH1750 m/s^2/sqrt(s)
(2.470794e-05/0.0167) #KVH1750 m/s^2/sqrt(hr)
1.434889e-05  #STIM300 (8 hours test) 33Hz bandwidth 10g
(1.434889e-05/0.0167) #STIM300 m/s^2/sqrt(hr)


sqrt((3.0 * avacc1y$av[20])/avacc1y$time[20])#KVH1750 & STIM300
1.391397e-05  #KVH1750 m/s^2/sqrt(s)
(1.391397e-05/0.0167) #KVH1750 m/s^2/sqrt(hr)
9.549511e-06 #STIM300 (8 hours test) 33Hz bandwidth 10g
(9.549511e-06/0.0167) #STIM300 m/s^2/sqrt(hr)

sqrt((3.0 * avacc1z$av[20])/avacc1z$time[20])#KVH1750 & STIM300
1.219521e-05 #KVH1750 m/s^2/sqrt(s)
(1.219521e-05/0.0167) #KVH1750 m/s^2/sqrt(hr)
1.436321e-05 #STIM300 (8 hours test) 33Hz bandwidth 10g
(1.436321e-05/0.0167) #STIM300 m/s^2/sqrt(hr)

#LAST STIM300 CONFIGURATION

#SERIAL NUMBER = N25581204825130
#
#PRODUCT = STIM300
#
#PART NUMBER = 84167-34000-321 REV B
#
#FW CONFIG = SWD11955 REV 6
#
#GYRO OUTPUT UNIT = [\0xb0/s] - ANGULAR RATE
#
#ACCELEROMETER OUTPUT UNIT = [g] - ACCELERATION
#
#INCLINOMETER OUTPUT UNIT = [g] - ACCELERATION
#
#SAMPLE RATE [samples/s] = 125
#
#GYRO CONFIG = XYZ
#
#ACCELEROMETER CONFIG = XYZ
#
#INCLINOMETER CONFIG = XYZ
#
#GYRO LP FILTER -3dB FREQUENCY, X-AXIS [Hz] = 33
#
#GYRO LP FILTER -3dB FREQUENCY, Y-AXIS [Hz] = 33
#
#GYRO LP FILTER -3dB FREQUENCY, Z-AXIS [Hz] = 33
#
#ACCELEROMETER LP FILTER -3dB FREQUENCY, X-AXIS [Hz] = 33
#
#ACCELEROMETER LP FILTER -3dB FREQUENCY, Y-AXIS [Hz] = 33
#
#ACCELEROMETER LP FILTER -3dB FREQUENCY, Z-AXIS [Hz] = 33
#
#INCLINOMETER LP FILTER -3dB FREQUENCY, X-AXIS [Hz] = 16
#
#INCLINOMETER LP FILTER -3dB FREQUENCY, Y-AXIS [Hz] = 16
#
#INCLINOMETER LP FILTER -3dB FREQUENCY, Z-AXIS [Hz] = 16
#
#AUX LP FILTER -3dB FREQUENCY [Hz] = 16
#
#AUX COMP COEFF: A = 1.0000000e+00, B = 0.0000000e+00
#
#DATAGRAM = RATE,ACCELERATION,INCLINATION,TEMPERATURE
#
#DATAGRAM TERMINATION = NONE
#
#BIT-RATE [bits/s] = 921600
#
#DATA LENGTH = 8
#
#STOP BITS = 1
#
#PARITY = NONE
#
#LINE TERMINATION = ON

