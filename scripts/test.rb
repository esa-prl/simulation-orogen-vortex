require 'orocos'
require 'readline'

include Orocos
Orocos.initialize

Orocos.conf.load_dir('../../../../bundles/rover/config/orogen/')

Orocos.run 'vortex::Task' => 'vortex' do

	vortex = Orocos.name_service.get 'vortex'
  	Orocos.conf.apply(vortex, ['exoter'], :override => true)
  	vortex.configure

	vortex.start
	Readline::readline("Press ENTER to exit\n")

end
