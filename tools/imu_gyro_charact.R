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

#load("stim300_gyro_16bnw_500hz_analysis.Rdata")
#load("stim300_gyro_16bnw_250hz_analysis.Rdata")
#load("stim300_gyro_262bnw_500hz_analysis.Rdata")

#############
# TEST Gyro
#############
#values <- read.table ("kvh_imu_gyro.0.data", sep=" ")
values <- read.table ("stim300_imu_gyro.0.data", sep=" ")

names(values) = c("time", "gyrox", "gyroy", "gyroz")

str(values)

#Time is in microseconds (10^-6)
delta <- diff(values$time)
delta <- mean(delta)/1000000

########################## X Axis ##########################
imu <- ts(as.numeric(array(na.omit(values$gyrox))), deltat=mean(delta))

#Frequency
frequency (imu)

#### Calculating the Allan Variance for the gyroscope ####
avgyro1x <- avar (imu@.Data, frequency (imu))

########################## Y Axis ##########################
imu <- ts(as.numeric(array(na.omit(values$gyroy))), deltat=mean(delta))

#Frequency
frequency (imu)

#### Calculating the Allan Variance for the gyroscope ####
avgyro1y <- avar (imu@.Data, frequency (imu))

########################## Z Axis ##########################
imu <- ts(as.numeric(array(na.omit(values$gyroz))), deltat=mean(delta))

#Frequency
frequency (imu)

#### Calculating the Allan Variance for the gyroscope ####
avgyro1z <- avar (imu@.Data, frequency (imu))

#### Plotting the results ####
x11()
plotCI (x=avgyro1x$time, y=sqrt(avgyro1x$av), uiw =  sqrt(avgyro1x$av)*avgyro1x$error, xaxt="n", yaxt="n", pch=0, gap=0, col="red", slty=par("lty.2") , log= "xy", xlab="", ylab="")
lines (avgyro1x$time,sqrt(avgyro1x$av), col="red")
plotCI (avgyro1y$time,sqrt(avgyro1y$av),uiw =  sqrt(avgyro1y$av)*avgyro1y$error,log= "xy", xaxt="n", yaxt="n", pch=8, gap=0, col="green", xlab="", ylab="", add=TRUE)
lines (avgyro1y$time,sqrt(avgyro1y$av), col="green")
plotCI (avgyro1z$time,sqrt(avgyro1z$av),uiw =  sqrt(avgyro1y$av)*avgyro1y$error, log= "xy", xaxt="n", yaxt="n", pch=17, gap=0,  col="blue", xlab="", ylab="", add=TRUE)
lines (avgyro1z$time,sqrt(avgyro1z$av), col="blue")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=2.0, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (rad/s)")

legend(10, 1e-04, c("Gyroscope X", "Gyroscope Y", "Gyroscope Z"),  col = c("red", "green", "blue"), pch=c(0, 8, 17))

#dev.print(png, file="xsens_gyro.png", width=1024, height=768, bg = "white") # To save the x11 device in a png
plotCI (x=avgyro1x$time, y=sqrt(avgyro1x$av), uiw =  sqrt(avgyro1x$av)*avgyro1x$error, xaxt="n", yaxt="n", pch=0, gap=0, col="red", slty=par("lty.2") , log= "xy", xlab="", ylab="")
lines (avgyro1x$time,sqrt(avgyro1x$av), col="red")
points (avgyro1y$time,sqrt(avgyro1y$av), log= "xy", xaxt="n", yaxt="n", pch=8, col="green", xlab="", ylab="", add=TRUE)
lines (avgyro1y$time,sqrt(avgyro1y$av), col="green")
points (avgyro1z$time,sqrt(avgyro1z$av), log= "xy", xaxt="n", yaxt="n", pch=17, col="blue", xlab="", ylab="", add=TRUE)
lines (avgyro1z$time,sqrt(avgyro1z$av), col="blue")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=2.0, col="orange")
title(main = "Allan variance Analysis", xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (rad/s)")

legend(10, 1e-04, c("Gyroscope X", "Gyroscope Y", "Gyroscope Z"),  col = c("red", "green", "blue"), pch=c(0, 8, 17))

#### Plotting the results in a file ####
png(filename = "stim300_gyro_allanvar.png", width=1024, height=768, units = "px", pointsize = 22, bg = "white", res = NA)
plotCI (x=avgyro1x$time, y=sqrt(avgyro1x$av), uiw =  sqrt(avgyro1x$av)*avgyro1x$error, xaxt="n", yaxt="n", pch=0, gap=0, col="red", slty=par("lty.2") , log= "xy", xlab="", ylab="")
lines (avgyro1x$time,sqrt(avgyro1x$av), col="red")
plotCI (avgyro1y$time,sqrt(avgyro1y$av),uiw =  sqrt(avgyro1y$av)*avgyro1y$error,log= "xy", xaxt="n", yaxt="n", pch=8, gap=0, col="green", xlab="", ylab="", add=TRUE)
lines (avgyro1y$time,sqrt(avgyro1y$av), col="green")
plotCI (avgyro1z$time,sqrt(avgyro1z$av),uiw =  sqrt(avgyro1y$av)*avgyro1y$error, log= "xy", xaxt="n", yaxt="n", pch=17, gap=0,  col="blue", xlab="", ylab="", add=TRUE)
lines (avgyro1z$time,sqrt(avgyro1z$av), col="blue")
axis(1, c(0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000, 100000))
axis(2, c(0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 0, 1, 10, 100, 1000, 10000))
grid(equilogs=TRUE, lwd=2.0, col="orange")
title(xlab = "Cluster Times (Sec)", ylab = "Allan Standard Deviation (rad/s)", prob=TRUE,
     cex.lab=1.5, cex.axis=1.5, cex.main=1.5, cex.sub=1.5)

legend(10, 1e-04, c("Gyroscope X", "Gyroscope Y", "Gyroscope Z"),  col = c("red", "green", "blue"), pch=c(0, 8, 17))
dev.off()

### Delete all values
rm (values, imu)

#### Save the result in a R image
#save.image (file = "stim300_gyro_16bnw_500hz_analysis.Rdata")
#save.image (file = "stim300_gyro_16bnw_250hz_analysis.Rdata")
#save.image (file = "stim300_gyro_262bnw_500hz_analysis.Rdata")

#######################################
#### Calculate the values (STIM300 Gyro)#
#######################################
plotav (avgyro1x)

##
#Random Walk, can be directly obtained by reading the slope line at T = 1
##
#Test 1 #
rwtodeghr <- function(rad_s)
{
    rad_hr = (rad_s * (180/pi))*sqrt(3600)
    return(rad_hr)
}


approx (x=c(avgyro1x$time[7], avgyro1x$time[8]), y= c(sqrt(avgyro1x$av[7]), sqrt(avgyro1x$av[8])), n=100)

#COMPARISON WITH OTHERS
3.087787e-06 #KVH1750 (5 hours test)
3.070302e-06 #KVH1750 (8 hours test)
4.624311e-05 #STIM300 (5 hours test) 33Hz bandwidth 10g Dagon filter on
4.655430e-05 #STIM300 (8 hours test) 33Hz bandwidth 10g Dagon
4.320343e-05 #STIM300 (5 hours test)
4.274725e-05 #STIM300 rad/s/sqrt(Hz) datasheet 
6.349347e-05 #iMAR rad/s/sqrt(Hz) datasheet 
0.0006643596 #XSens rad/s/sqrt(Hz)

approx (x=c(avgyro1y$time[7], avgyro1y$time[8]), y= c(sqrt(avgyro1y$av[7]), sqrt(avgyro1y$av[8])), n=100)

#COMPARISON WITH OTHERS
3.388457e-06 #KVH1750 (5 hours test)
3.316984e-06 #KVH1750 (8 hours test)
4.400006e-05 #STIM300 (5 hours test) 33Hz bandwidth 10g Dagon filter on
4.384513e-05 #STIM300 (8 hours test) 33Hz bandwidth 10g Dagon
4.175001e-05 #(5 hours test)
4.268059e-05 #STIM300 rad/s/sqrt(Hz)
1.049867e-04 #iMAR rad/s/sqrt(Hz)
0.0006978859 #rad/s/sqrt(Hz)

approx (x=c(avgyro1z$time[7], avgyro1z$time[8]), y= c(sqrt(avgyro1z$av[7]), sqrt(avgyro1z$av[8])), n=100)

#COMPARISON WITH OTHERS
3.554119e-06 #KVH1750 (5 hours test)
3.004417e-06 #KVH1750 (8 hours test)
4.275989e-05 #STIM300 (5 hours test) 33Hz bandwidth 10g Dagon filter on
4.298269e-05 #STIM300 (8 hours test) 33Hz bandwidth 10g Dagon
4.060973e-05 #(5 hours test)
4.319166e-05 #STIM300 rad/s/sqrt(Hz)
7.936806e-05 #iMAR rad/s/sqrt(Hz)
0.0006390777 #rad/s/sqrt(Hz)

#DATASHEET for KVH1750
0.012 #deg/sqrt(hr)
(0.012*2.9e-04) #rad/s/sqrt(Hz) or rad/sqrt(s)

#DATASHEET for STIM300
0.15 #deg/sqrt(hr)
(0.15*2.9e-04) #rad/s/sqrt(Hz) ot rad/sqrt(s)

##
#Bias Instability, can be directly obtained by reading the 0 slope line 
##

biastodeghr <- function(rad_s)
{
    rad_hr = (rad_s * (180/pi))*3600
    return(rad_hr)
}

#Test 1 Bias instability Coeff
sqrt(avgyro1x$av[16])/(sqrt(2*log(2)/pi)) #KVH1750
sqrt(avgyro1x$av[16])/(sqrt(2*log(2)/pi)) #STIM300

#COMPARISON WITH OTHERS
5.438596e-07 #KVH1750 (5 hours test)
2.855782e-07 #KVH1750 (8 hours test)
1.706126e-05 #STIM300 (5 hours test) 33Hz bandwidth 10g Dagon filter on
7.170674e-06 #STIM300 (8 hours test) 33Hz bandwidth 10g Dagon
6.882191e-06 #(5 hours test)
4.697074e-06 #STIM300 rad/s datasheet is 0.5 deg/hr 0.016 deg/hr
1.857284e-06 #iMAR rad/s datasheet
0.0001463489 #XSens rad/s

sqrt(avgyro1y$av[16])/(sqrt(2*log(2)/pi)) #KVH1750
sqrt(avgyro1y$av[16])/(sqrt(2*log(2)/pi)) #STIM300

#COMPARISON WITH OTHERS
1.054195e-06 #KVH1750 (5 hours test)
5.230224e-07 #KVH1750 (8 hours test)
3.761969e-05 #STIM300 (5 hours test) 33Hz bandwidth 10g Dagon filter on
7.549352e-06 #STIM300 (8 hours test) 33Hz bandwidth 10g Dagon
8.479295e-06 #(5 hours test)
8.841014e-06 #STIM300 rad/s
6.456726e-06 #iMAR rad/s
0.000151518 #XSens rad/s

sqrt(avgyro1z$av[16])/(sqrt(2*log(2)/pi))#KVH2750
sqrt(avgyro1z$av[15])/(sqrt(2*log(2)/pi))#STIM300

#COMPARISON WITH OTHERS
5.370595e-07 #KVH1750 (5 hours test)
2.385443e-07 #KVH1750 (8 hours test)
1.188613e-05 #STIM300 (5 hours test) 33Hz bandwidth 10g Dagon filter on
7.530615e-06 #STIM300 (8 hours test) 33Hz bandwidth 10g Dagon
5.591609e-06 #STIM300 (5 hours test)
6.71548e-06 #STIM300 rad/s which is 1.3 deg/hr
1.459917e-05 #iMAR rad/s
0.0001488385 #XSens rad/s


##
#Rate Random Walk can be obtained by reading the allan variance value
#by a slope at +1/2. K=sqrt(3)*allandeviation(t)/sqrt(t)
# or K=sqrt((3*allandeviation(t))/sqrt)t))
##
sqrt((3.0 * avgyro1x$av[20])/avgyro1x$time[20])#KVH1750 & STIM300
7.097969e-09 #KVH1750 rad/s/sqrt(s)
(7.097969e-09 / 2.9e-04)*3600 #KVH1750 deg/hr/sqrt(hr)
2.924086e-07  #STIM300 (8 hours test) 33Hz bandwidth 10g rad/s/sqrt(s)
(2.924086e-07 / 2.9e-04)*3600  #STIM300 deg/hr/sqrt(hr)

sqrt((3.0 * avgyro1y$av[20])/avgyro1y$time[20])#KVH1750 & STIM300
5.193316e-08  #KVH1750 rad/s/sqrt(s)
(5.193316e-08 / 2.9e-04)*3600 #KVH1750 rad/s/sqrt(hr)
2.591558e-07 #STIM300 (8 hours test) 33Hz bandwidth 10g
(2.591558e-07 / 2.9e-04)*3600 #STIM300 deg/hr/sqrt(hr)

sqrt((3.0 * avgyro1z$av[20])/avgyro1z$time[20])#KVH1750 & STIM300
9.61242e-09 #KVH1750 rad/s/sqrt(s)
(9.61242e-09 / 2.9e-04)*3600 #KVH1750 rad/s/sqrt(hr)
5.058095e-08 #STIM300 (8 hours test) 33Hz bandwidth 10g
(5.058095e-08 / 2.9e-04)*3600 #STIM300 deg/hr/sqrt(hr)


