#! /usr/bin/env ruby

require 'orocos'
require 'orocos/log'
require 'vizkit'

include Orocos

if ARGV.size < 1 then 
    puts "usage: process_logs.rb <data_log_directory>"
    exit
end

#Initializes the CORBA communication layer
Orocos.initialize

Orocos.run('orientation_estimator_test', 'ikf_orientation_estimator_test', 'ukf_orientation_estimator_test') do 
  
    # log all the output ports
    Orocos.log_all_ports 
    Orocos.conf.load_dir('../config')
    
    # get the invidual tasks
    attitude_task = TaskContext.get 'orientation_estimator'
    Orocos.conf.apply(attitude_task, ['default'], :override => true)
    ikf_attitude_task = TaskContext.get 'ikf_orientation_estimator'
    Orocos.conf.apply(ikf_attitude_task, ['default'], :override => true	)
    ukf_attitude_task = TaskContext.get 'ukf_orientation_estimator'
    Orocos.conf.apply(ukf_attitude_task, ['default'], :override => true	)
     
     # connect the tasks to the logs
    log_replay = Orocos::Log::Replay.open( ARGV[0] ) 
    
#     view3d = Vizkit.default_loader.create_widget('vizkit::QVizkitWidget')
#     view3d.setParent(self)
# 
#     vizkit_rbs = view3d.createPlugin('RigidBodyStateVisualization')
#     vizkit_rbs.resetModel(0.5)
#     vizkit_rbs.displayCovarianceWithSamples(true)
    
    #Mapping the inputs ports in the orientation task
    log_replay.dsp3000.rotation.connect_to( attitude_task.fog_samples, :type => :buffer, :size => 10 )
    log_replay.xsens_imu.orientation_samples.connect_to( attitude_task.imu_orientation, :type => :buffer, :size => 10 )
 
    #Mapping the inputs ports in the ikf orientation task
    log_replay.xsens_imu.calibrated_sensors.connect_to( ikf_attitude_task.imu_samples, :type => :buffer, :size => 10 )
    log_replay.dsp3000.rotation.connect_to( ikf_attitude_task.fog_samples, :type => :buffer, :size => 10 )
    log_replay.xsens_imu.orientation_samples.connect_to( ikf_attitude_task.imu_orientation, :type => :buffer, :size => 10 ) do |sample|
#             vizkit_rbs.updateRigidBodyState(sample)
# 	sample.cov_position.data[0] = 5
	  sample      
    end
    
    
    #Mapping the inputs ports in the ukf orientation task
    log_replay.xsens_imu.calibrated_sensors.connect_to( ukf_attitude_task.imu_samples, :type => :buffer, :size => 10 )
    log_replay.dsp3000.rotation.connect_to( ukf_attitude_task.fog_samples, :type => :buffer, :size => 10 )
    log_replay.xsens_imu.orientation_samples.connect_to( ukf_attitude_task.imu_orientation, :type => :buffer, :size => 10 ) do |sample|
#             vizkit_rbs.updateRigidBodyState(sample)
# 	sample.cov_position.data[0] = 5
	  sample      
    end
    
#     view3d.show
    
    attitude_task.configure
    attitude_task.start
     
    ikf_attitude_task.configure
    ikf_attitude_task.start

    ukf_attitude_task.configure
    ukf_attitude_task.start
    
    
#     Vizkit.display attitude_task.attitude_b_g,:orientation => "degrees",:widget=>"Plot2d"

     # open the log replay widget
    control = Vizkit.control log_replay
    control.speed = 1 #4
#     control.auto_replay
    Vizkit.exec
    
    

    
end