#! /usr/bin/env ruby

require 'orocos'
require 'orocos/log'
require 'vizkit'

include Orocos

if ARGV.size < 0 then 
    puts "usage: run_ikf_stim300.rb"
    exit
end

#Initializes the CORBA communication layer
Orocos.initialize

proprio = {:proprio => true}

Orocos.run('ikf_orientation_estimator_test') do 
  
    # log all the output ports
    Orocos.log_all_ports 
    Orocos.conf.load_dir('../config')
    
    # get the invidual tasks
    ikf_attitude_task = TaskContext.get 'ikf_orientation_estimator'
    Orocos.conf.apply(ikf_attitude_task, ['stim300'], :override => true	)
    
    if proprio[:proprio]
	proprio = TaskContext.get 'propriocessing'

	#Mapping the inputs ports in the ikf orientation task
	proprio.calibrated_sensors_out.connect_to( ikf_attitude_task.imu_samples, :type => :buffer, :size => 10 )
    else
	stim300 = TaskContext.get 'stim300'
	
	#Mapping the inputs ports in the ikf orientation task
	stim300.calibrated_sensors.connect_to( ikf_attitude_task.imu_samples, :type => :buffer, :size => 10 )
    end

    ikf_attitude_task.configure
    ikf_attitude_task.start

    loop do        
	sleep 0.01
    end
    
end