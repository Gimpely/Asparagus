**** 08.06.22 *****

Zagon programa za premikanje robota po točkah.

1) Zaženemo "roscore"
2) Zaženemo "rosrun beckhoff_comm plc_pc_comm" s tem se vzpostavi komunikacija med beckhoff krmilnikom in ros pc-jem
3) Zaženemo "rosrun delta_robot c_moveL_control.py". Odpre se nam okno, ki kaže nastavljeno pozicijo robota. Pozicijo spreminjamo s številkami 1,2,3... Robot se premakne na shranjeno pozicijo.
4) Zaženemo "rosrun delta_robot control_r2omegas.py". V tej kodi teče PD regulator za robota.




**** 30.06.22 *****

Zagon programa za pemikanje robota z joystickom

1) Zaženemo "roscore"
2) Zaženemo "rosrun beckhoff_comm plc_pc_comm" s tem se vzpostavi komunikacija med beckhoff krmilnikom in ros pc-jem
3) Zaženemo "rosrun joy joy_node" s tem se vzpostavi povezava z joystickom
4) Zaženemo "rosrun delta_robot joy_control.py" s tem beremo ukaz iz joysticka
5) Zaženemo "rosrun delta_robot control_r2omegas.py". V tej kodi teče PD regulator za robota.


**** 1.07.22 *****

Povezava na računalnik brez monitorja.
https://kb.nomachine.com/AR03P00973

**** 13.07.22 *****

Zagon laserskega skenerja Sick LMS111 in programov za generiranje 3D slike in pretvorbo v world koordinatni sistem

1) Zaženemo "roslaunch sick_scan sick_lms_1xx.launch hostname:=192.168.1.100" s tem se vzpostavi komunikacija za laserskim skenerjem
2) Zaženemo "rosrun transformations sensor_to_robot.py laser 0.7 0.05 0.5 0 2.36 0" da dobimo statično transformacijo med koordinatnim sistemom laserja in robota
3) Zaženemo "rosrun beckhoff_comm plc_pc_comm" s tem se vzpostavi komunikacija med beckhoff krmilnikom in ros pc-jem
4) Zaženemo "rosrun joy joy_node" s tem se vzpostavi povezava z joystickom
5) Zaženemo "rosrun tracks joy_control.py" s tem se vzpostavi povezava z joystickom za premikanje gosenic
6) Zaženemo "rosrun transformations robot_to_global.py" da dobimo dinamično transformacijo med koordinatnim sistemom robota in globalnim sistemom
7) Zaženemo "rosrun delta_robot sick_scanner.py" program pretvarja koordinate v robot cs
8) Zaženemo "rosrun delta_robot get_img_from_scanner2.py" program sestavlja sliko skenerja v globalnem koordinatnem sistemu


**** 21.07.22 *****

Lokacija ros knjižnjic: /opt/ros/noetic/lib/python3/dist-packages
Zagon vizualizacije za intel realsense kamere: "realsense-viewer"
Lokacijo launch datoteke poiščemo z ukazom "roscd ime_datoteke.launch"
Zagon intel realsense v ros: "roslaunch realsense2_camera rs_camera.launch"
Pretvorba iz koordinatnega sistema realsense kamere v ks robota: "rosrun transformations sensor_to_robot.py camera_depth_optical_frame 0.75 0.1 0.4 2.37 0 -1.57"
Shranjevanje podatkov z rosbag: "rosbag record -O L515_global_points.bag --duration=10 /intel_l515_global_cloud"

**** 27.07.22 ****
Filter za laserski skener: http://wiki.ros.org/laser_filters/Tutorials/Laser%20filtering%20using%20the%20filter%20nodes

**** 29.07.22 ****

Iskanje točk pobiranja iz rosbag datoteke. Uporabljamo linijski laserski skener Sick lms111

1) Zaženemo "roscore"
2) Zaženemo "rosrun transformations sensor_to_robot.py laser 0.7 0.05 0.5 0 2.36 0", da transformiramo koordinatni sistem iz skenerja v koordinatni sistem robota
3) Zaženemo "rosrun transformations robot_to_global.py", da transformiramo koordinatni sistem robota v globalni (fiksni) koordinatni sistem. Za transformacijo se upoštevajo podatki z odometrije vozička.
4) Zaženemo "rosrun acquire_asparagus sick_scanner.py". V tem programu beremo podatke iz senzorja, jih pretvarjamo v koordinatni sistem robota in v tem koordinatnem sistemu jih obrežemo po Y osi, da odstranimo gosenice iz vidnega polja.
5) Zaženemo "rosrun acquire_asparagus sick_get_global_cloud.py". Ta program zlaga 3D model iz linijskih točk, ko se voziček premika.
6) Zaženemo "rosrun acquire_asparagus detect_asparagus.py". Ta program bere globalni point cloud. Najprej odrežemo vse točke, ki predstavljajo tla in vse točke, ki so višje recimo od 10 cm. Tako dobimo točke, kjer se šparglji dotikajo tal. Sledi korak iskanje 2D histograma. Naše območje med gosenicami razdelimo na polja in v vsakem polju pogledamo število točk. V tistih poljih, kjer je število točk največje predvidevamo, da se nahaja špargelj. Znotraj vsakega polja, kjer je določeno število točk poiščemo mediano točk po x in po y. To storimo za vsa polja, ki imajo število točk nad mejo. Znotraj polja pogledamo tudi, če so koordinate po Z blizu tal. Tako določimo ali se tam špargelj dotika tal. V naslednjem koraku poiščemo ali je špargelj dovolj visok za obiranje. To storimo tako, da iščemo visoke točke (z koordinata) znotraj kroga, kjer se šparglji dotikajo tal. V zadnjem koraku določimo še točke pobiranja, ki so določene na fiksni Z višini.
7) Za testiranje lahko zaženemo rosbag datoteko "rosbag play scan_2.bag"


**** 08.08.22 ****

Kaj je potrebno upoštevati pri planiranju poti robota:
- Kateri šparglji so znotraj delovnega območja robota.
- Kateri šparglji so bili prvi zaznani (tiste je potrebno najprej pobrati)
- Višino šparglja
- Ali raste več špargljev v bližini (prilagajanje kota pobiranja)
- Ali je kakšen manjši špargelj ovira gibanje robota
- Prijemalo robota simuliramo s krogom (gledamo, če je znotraj kroga kakšen špargelj ovira)

Postopek pobiranja:roslaunch sick_scan sick_lms_1xx.launch hostname:=192.168.1.100

**** 12.08.22 ****

Zagon programov za pobriranje špargljev:

1) roslaunch sick_scan sick_lms_1xx.launch hostname:=192.168.1.100
2) rosrun transformations sensor_to_robot.py laser 0.57 0.04 0.5 0 2.36 0
3) rosrun transformations robot_to_global.py
4) rosrun beckhoff_comm plc_pc_comm
5) rosrun joy joy_node
6) rosrun tracks joy_control.py
7) rosrun delta_robot control_r2omegas.py
8) rosrun acquire_asparagus sick_scanner.py
9) rosrun acquire_asparagus sick_get_global_cloud.py
10) rosrun acquire_asparagus detect_asparagus.py
11) rosrun delta_robot motion_planning.py










