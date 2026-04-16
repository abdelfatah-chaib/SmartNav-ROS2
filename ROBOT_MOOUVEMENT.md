# SmartNav-ROS2

Digital Twin and Simulation for the SmartNav Autonomous Cane

## Pourquoi le robot ne bougeait pas avant

Le problème ne venait pas du clavier `teleop_twist_keyboard` lui-même.
Les commandes étaient bien publiées sur le topic ROS 2 `/cmd_vel`, mais elles
ne produisaient aucun mouvement dans Gazebo parce que la chaîne complète de
commande n'était pas cohérente.

Les principaux problèmes étaient les suivants :

1. Le plugin de mouvement initialement utilisé ne correspondait pas bien à
   l'installation Gazebo disponible.
2. La configuration du bridge ROS 2 <-> Gazebo n'était pas alignée avec le
   topic réellement consommé par le plugin de mouvement.
3. La syntaxe de certains bridges `ros_gz_bridge` était incorrecte.
4. Le paramètre `robot_description` du launch file devait être forcé comme
   chaîne de caractères avec ROS 2 Jazzy.

En pratique, cela donnait la situation suivante :

- `teleop_twist_keyboard` publiait bien sur `/cmd_vel`
- `ros2 topic echo /cmd_vel` affichait bien des messages `Twist`
- mais Gazebo ne transformait pas ces messages en déplacement réel du robot

## Pourquoi cela marche maintenant

Le projet a été corrigé pour que toute la chaîne soit cohérente :

1. Le robot utilise maintenant le plugin `VelocityControl`, disponible dans la
   stack Gazebo installée avec ROS 2 Jazzy.
2. Le bridge `cmd_vel` pointe maintenant vers le bon topic Gazebo, cohérent
   avec la configuration du plugin.
3. La syntaxe des bridges a été corrigée avec le format :

```text
/topic@type_ros@type_gz
```

4. Le paramètre `robot_description` est déclaré explicitement comme `str`, ce
   qui évite le crash du lancement sous Jazzy.

Maintenant la chaîne fonctionne ainsi :

```text
teleop_twist_keyboard
        |
        v
      /cmd_vel   (ROS 2)
        |
        v
   ros_gz_bridge
        |
        v
      /cmd_vel   (Gazebo)
        |
        v
VelocityControl applique la vitesse sur le robot
```

## Fichiers corrigés

- `smartnav_description/urdf/smartnav.xacro`
- `smartnav_core/launch/start_simulation.launch.py`
- `smartnav_gazebo/worlds/smartnav.world`

## Lancement correct

Dans un premier terminal :

```bash
cd /home/zineb/SmartNav-ROS2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select smartnav_description smartnav_core smartnav_gazebo
source /home/zineb/SmartNav-ROS2/install/setup.bash
ros2 launch smartnav_core start_simulation.launch.py
```

Dans un deuxième terminal :

```bash
source /opt/ros/jazzy/setup.bash
source /home/zineb/SmartNav-ROS2/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Vérification rapide

Pour vérifier que les commandes sont bien publiées :

```bash
ros2 topic echo /cmd_vel
```

Si le robot ne bouge plus dans le futur, vérifier en priorité :

- que Gazebo n'est pas en pause
- que le bon workspace est sourcé :
  `/home/zineb/SmartNav-ROS2/install/setup.bash`
- que le topic `/cmd_vel` contient bien des messages `Twist`
- que le launch utilisé est bien `smartnav_core/start_simulation.launch.py`
