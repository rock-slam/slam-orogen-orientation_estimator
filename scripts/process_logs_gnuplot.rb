#! /usr/bin/env ruby

require 'orocos'
require 'orocos/log'
require 'vizkit'
require 'plotData'
require 'quaternion'

include Orocos

if ARGV.size < 1 then 
    puts "usage: process_logs.rb ru"
    exit
end

def positionRbs ( data ) 
  position = [data.position.x, data.position.y, data.position.z	] 
  position
end 

def errorRbs ( data ) 
  error = [2*Math.sqrt(data.cov_position.data[0]),2*Math.sqrt(data.cov_position.data[4])]
  error
end 

def standartDeviation ( data ) 
  standart_deviation = [Math.sqrt(data.data[0]),Math.sqrt(data.data[4]),Math.sqrt(data.data[8])]
  standart_deviation
end 

def register3Axis ( title, x_axis, y_axis ) 
    plot = DataPlot.new()	
    plot.register2D( :x, {:title => "roll", :lt =>"l lt 1"} )
    plot.register2D( :y, {:title => "pitch", :lt =>"l lt 2"} )
    plot.register2D( :z, {:title => "yaw", :lt =>"l lt 3"} )
    plot.setTitle(title, "Helvetica,14")
    plot.setXLabel(x_axis, "Helvetica,14")
    plot.setYLabel(y_axis, "Helvetica,14")
    plot
end 

def dt?( data ) 
    if @init_time == 0.0 
	@init_time = data.time.to_f
    end
    dt = data.time.to_f - @init_time
    dt
end


def register3ExtraAxis(plot, name) 
    plot.register2D( :a, {:title => "roll #{name}", :lt =>"l lt 4"} )
    plot.register2D( :b, {:title => "pitch #{name}", :lt =>"l lt 5"} )
    plot.register2D( :c, {:title => "yaw #{name}", :lt =>"l lt 6"} )
end

def register3ExtraAxisTwo(plot, name) 
    plot.register2D( :u, {:title => "roll #{name}", :lt =>"l lt 7"} )
    plot.register2D( :v, {:title => "pitch #{name}", :lt =>"l lt 8"} )
    plot.register2D( :w, {:title => "yaw #{name}", :lt =>"l lt 9"} )
end

def plotArrow(plot, data)
    quaternion = Quaternion.new(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z)
    pos = positionRbs( data )
    dcm = quaternion.q_to_dcm
    plot.arrow(pos,[pos[0]+dcm[0,0] *0.2,pos[1]+dcm[1,0] *0.2, pos[2]]) 
end

BASE_DIR = File.expand_path('..', File.dirname(__FILE__))
ENV['PKG_CONFIG_PATH'] = "#{BASE_DIR}/build:#{ENV['PKG_CONFIG_PATH']}"

Orocos.initialize

    # log all the output ports
    Orocos.log_all_ports 

    # get the invidual tasks
    log_replay = Orocos::Log::Replay.open( ARGV[0]) # This log_re[play is for the logs in the folder
    
    log_replay1 = Orocos::Log::Replay.open( ARGV[1]) # This loag replay is for the log to test (indivisual log)

    plot_orientation = register3Axis( ARGV[0], "Time (s)", "Orientation (deg)")
    plot_key = Array.new
    plot_key << :x
    plot_key << :y
    plot_key << :z
    
    register3ExtraAxis(plot_orientation,"Xsens")
    plot_extra_key = Array.new
    plot_extra_key << :a
    plot_extra_key << :b
    plot_extra_key << :c
    
    register3ExtraAxisTwo(plot_orientation,"New")
    plot_extra_key = Array.new
    plot_extra_key << :u
    plot_extra_key << :v
    plot_extra_key << :w
    
    sample = 0 
   
    dt = 0.0 
    init_time = 0.0 
    
    
    #Log of the original orientation_estimator
    log_replay.orientation_estimator.attitude_b_g.connect_to :type => :buffer,:size => 2000 do|data,name|
      
      if init_time == 0.0 
	  init_time = data.time.to_f
	  
      end
      
      quaternion = Quaternion.new(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z)
      dt = data.time.to_f - init_time
      
      plot_orientation.addData(  :x, [dt, quaternion.to_pitch * 180 / Math::PI] ) if data
      plot_orientation.addData(  :y, [dt, quaternion.to_roll * 180 / Math::PI] ) if data
      plot_orientation.addData(  :z, [dt, quaternion.to_yaw * 180 / Math::PI] ) if data
      
      data
    
    end
    
    #log of the xsens imu
    log_replay.xsens_imu.orientation_samples.connect_to :type => :buffer,:size => 2000 do|data,name|
      
      if init_time == 0.0 
	  init_time = data.time.to_f
	  
      end
      
      quaternion = Quaternion.new(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z)
      dt = data.time.to_f - init_time
      
      plot_orientation.addData(  :a, [dt, quaternion.to_pitch * 180 / Math::PI] ) if data
      plot_orientation.addData(  :b, [dt, quaternion.to_roll * 180 / Math::PI] ) if data
      plot_orientation.addData(  :c, [dt, quaternion.to_yaw * 180 / Math::PI] ) if data
      
      data
    
    end
    
    log_replay.align( :use_sample_time )
    control = Vizkit.control log_replay
    
    ##############################################
    
    #log of the new orientation_estimator
    log_replay1.orientation_estimator.attitude_b_g.connect_to :type => :buffer,:size => 2000 do|data,name|
      
      if init_time == 0.0 
	  init_time = data.time.to_f
	  
      end
      
      quaternion = Quaternion.new(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z)
      dt = data.time.to_f - init_time
      
      plot_orientation.addData(  :u, [dt, quaternion.to_pitch * 180 / Math::PI] ) if data
      plot_orientation.addData(  :v, [dt, quaternion.to_roll * 180 / Math::PI] ) if data
      plot_orientation.addData(  :w, [dt, quaternion.to_yaw * 180 / Math::PI] ) if data
      
      data
    
    end
    

    
    log_replay1.align( :use_sample_time )
    control = Vizkit.control log_replay1
    
    Vizkit.exec
  
    plot_orientation.show()