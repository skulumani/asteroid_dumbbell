all: geographos geographos_weight golevka golevka_weight castalia castalia_cam castalia_weight castalia_cam_weight 52760 52760_cam 52760_weight 52760_cam_weight castalia_refinement castalia_refinement_cam castalia_refinement_weight castalia_refinement_cam_weight castalia_land castalia_land_cam

geographos:
	mkdir -p /tmp/videos/geographos
	PYTHONPATH=./ python dissertation/explore_plots.py -a ./data/exploration/kinematics/geographos.hdf5 /tmp/videos/geographos	

geographos_weight:
	mkdir -p /tmp/videos/geographos_weight
	PYTHONPATH=./ python dissertation/explore_plots.py -mw -a ./data/exploration/kinematics/geographos.hdf5 /tmp/videos/geographos_weight

golevka:
	mkdir -p /tmp/videos/golevka
	PYTHONPATH=./ python dissertation/explore_plots.py -a ./data/exploration/kinematics/golevka.hdf5 /tmp/videos/golevka	

golevka_weight:
	mkdir -p /tmp/videos/golevka_weight
	PYTHONPATH=./ python dissertation/explore_plots.py -mw -a ./data/exploration/kinematics/golevka.hdf5 /tmp/videos/golevka_weight

castalia:
	mkdir -p /tmp/videos/castalia
	PYTHONPATH=./ python exploration_sim.py -sa /tmp/videos/castalia ./data/exploration/explore/20180615_exploration_castalia_15000.hdf5 castalia

castalia_cam:
	mkdir -p /tmp/videos/castalia_cam
	PYTHONPATH=./ python exploration_sim.py -mc -sa /tmp/videos/castalia_cam ./data/exploration/explore/20180615_exploration_castalia_15000.hdf5 castalia

castalia_weight:
	mkdir -p /tmp/videos/castalia_weight
	PYTHONPATH=./ python exploration_sim.py -mw -sa /tmp/videos/castalia_weight ./data/exploration/explore/20180615_exploration_castalia_15000.hdf5 castalia

castalia_cam_weight:
	mkdir -p /tmp/videos/castalia_cam_weight
	PYTHONPATH=./ python exploration_sim.py -mc -mw -sa /tmp/videos/castalia_cam_weight ./data/exploration/explore/20180615_exploration_castalia_15000.hdf5 castalia

52760:
	mkdir -p /tmp/videos/52760
	PYTHONPATH=./ python exploration_sim.py -sa /tmp/videos/52760 ./data/exploration/explore/20180615_exploration_52760_15000.hdf5 52760

52760_cam:
	mkdir -p /tmp/videos/52760_cam
	PYTHONPATH=./ python exploration_sim.py -mc -sa /tmp/videos/52760_cam ./data/exploration/explore/20180615_exploration_52760_15000.hdf5 52760

52760_weight:
	mkdir -p /tmp/videos/52760_weight
	PYTHONPATH=./ python exploration_sim.py -mw -sa /tmp/videos/52760_weight ./data/exploration/explore/20180615_exploration_52760_15000.hdf5 52760

52760_cam_weight:
	mkdir -p /tmp/videos/52760_cam_weight
	PYTHONPATH=./ python exploration_sim.py -mc -mw -sa /tmp/videos/52760_cam_weight ./data/exploration/explore/20180615_exploration_52760_15000.hdf5 52760

castalia_refinement:
	mkdir -p /tmp/videos/castalia_refinement
	PYTHONPATH=./ python exploration_sim.py -lrsa /tmp/videos/castalia_refinement ./data/exploration/refine/20180619_castalia_refinement.hdf5 castalia

castalia_refinement_cam:
	mkdir -p /tmp/videos/castalia_refinement_cam
	PYTHONPATH=./ python exploration_sim.py -mc -lrsa /tmp/videos/castalia_refinement_cam ./data/exploration/refine/20180619_castalia_refinement.hdf5 castalia

castalia_refinement_weight:
	mkdir -p /tmp/videos/castalia_refinement_weight
	PYTHONPATH=./ python exploration_sim.py -mw -lrsa /tmp/videos/castalia_refinement_weight ./data/exploration/refine/20180619_castalia_refinement.hdf5 castalia

castalia_refinement_cam_weight:
	mkdir -p /tmp/videos/castalia_refinement_cam_weight
	PYTHONPATH=./ python exploration_sim.py -mc -mw -lrsa /tmp/videos/castalia_refinement_cam_weight ./data/exploration/refine/20180619_castalia_refinement.hdf5 castalia

castalia_land:
	mkdir -p /tmp/videos/castalia_land
	PYTHONPATH=./ python exploration_sim.py -lsa /tmp/videos/castalia_land ./data/exploration/land/20180619_castalia_landing.hdf5 castalia

castalia_land_cam:
	mkdir -p /tmp/videos/castalia_land_cam
	PYTHONPATH=./ python exploration_sim.py -mc -lsa /tmp/videos/castalia_land_cam ./data/exploration/land/20180619_castalia_landing.hdf5 castalia
