require 'orocos'
require 'readline'

include Orocos
Orocos.initialize

Orocos.conf.load_dir('../../../../bundles/rover/config/orogen/')

Orocos.run 'control', 'udp::Task' => 'udp' do

	# setup locomotion_control
    puts "Setting up locomotion_control"
    locomotion_control = Orocos.name_service.get 'locomotion_control'
    Orocos.conf.apply(locomotion_control, ['exoter'], :override => true)
    locomotion_control.configure
    puts "done"
	
	# setup udp
    puts "Setting up UDP"
	udp = Orocos.name_service.get 'udp'
  	Orocos.conf.apply(udp, ['exoter'], :override => true)
  	udp.configure
	puts "done"

	# setup read_joint_dispatcher
    puts "Setting up reading joint_dispatcher"
    read_joint_dispatcher = Orocos.name_service.get 'read_joint_dispatcher'
    Orocos.conf.apply(read_joint_dispatcher, ['exoter_reading'], :override => true)
    read_joint_dispatcher.configure
    puts "done"

  	# setup command_joint_dispatcher
    puts "Setting up commanding joint_dispatcher"
    command_joint_dispatcher = Orocos.name_service.get 'command_joint_dispatcher'
    Orocos.conf.apply(command_joint_dispatcher, ['exoter_commanding'], :override => true)
    command_joint_dispatcher.configure
    puts "done"


	puts "Connecting ports"

	udp.joints_readings.connect_to            			  read_joint_dispatcher.joints_readings
	command_joint_dispatcher.motors_commands.connect_to   udp.joints_commands
	
    read_joint_dispatcher.motors_samples.connect_to 	  locomotion_control.joints_readings
	locomotion_control.joints_commands.connect_to         command_joint_dispatcher.joints_commands

	udp.start
	sleep 1
	read_joint_dispatcher.start
    command_joint_dispatcher.start
	locomotion_control.start

	Readline::readline("Press ENTER to exit\n")

end
