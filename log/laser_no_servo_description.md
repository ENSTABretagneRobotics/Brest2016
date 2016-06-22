# Description du test du laser sans controle servo

## Filtrage des rosbag

On a du filtrer les rosbag:
```bash
rosbag filter laser_no_servo_2016-06-20-17-34-38.bag laser_no_servo_test4_maxDist_both_ins_range.bag '  
topic == "/gps" or topic == "/imu_boat" or topic == "/range_data" or topic == "/imu_laser"'
... sdf
```

Pour simuler les test lancer le roslaunc laser_complet en commentant les noeuds **arduino**, **gps** et **imu**
et lancer le noeud **LL_to_local**

## TEST 0
ras

## TEST 1
- longtemps a demarer
- galere sur la fin
- on commence a 7m et on recule
- a la fin on s'abaisse ca explique les fausses distances

## TEST 2
- trop bas donc voit le sol
- vers la fin ca va mieux

## TEST 3
- on a eviter d'etre trop bas
- on voit bien la difference de distance

## TEST 4
- on galere trop loin donc on chope pas le mur
- a partir de 30 ca marche niquel
- faut bien viser
- donnees coherentes
