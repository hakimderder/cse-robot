# Réunion du 26 février 2021

## Points discutés

- Odométrie pour ROS
- Déplacement autonome en extérieur
- Etudier les prblématiques
- Définir les objectifs
- Définir des capteurs GPS
- Place de travail à disposition
- Informations générales et déreoulement
- 


# Réunion du 18 mars 2021

## Points discutés

- Suivis du cours sur TheConstructSim (échéance au 11 avril 2021)
- Documentation ROS à lire
- Déterminer les étapes du projet (planning)
- 


# Réunion du 15 avril 2021

## Points discutés

- Cours suivi sur TheConstructSim
- Hiérarchie du git agribot
- Contacter Gabriel pour discussion sur l'environnement ROS et la RPI4
- Demander une carte SD pour faire un déploiement de test sur la RPI4 de prêt
- Faire tutoriels sur robot_reds
- 


# Réunion du 30 avril 2021

## Points discutés

- Créer un journal de travail
- Tests et déploiement sur la RPI4
- Présentation de ROS dans le rapport (éventuellement tuto annexe)
- Se renseigner sur GPS - RTK - IMU
- 


# Réunion du 20 mai 2021

## Points discutés

- Pause de 4 semaines pour terminer le semestre
- Proposer des dates de meeting pour début juillet avec Sébastien Guillaume pour obtenir des renseignements sur les corrections de positionnements
- Proposer un planning pour le prochain meeting
- Se renseigner sur GPS - RTK
-  


# Réunion du 24 juin 2021

## Proposition de planning

A raison de 24h-28h par semaine

Juin :
- S25 (10h) : Méthodes de positionnement de précision

Juillet :
- S26 (28h) : Méthodes de positionnement de précision + Rapport
- S27 (28h) : Rapport 
- S28 (28h) : Rapport + Méthodes de positionnement de précision
- S29 (24h) : Etude navigation ROS + Analyse
- S30 (24h) : Navigation ROS + Communication, SMS

Août :
- S31 (24h) : Intégration GPS
- S32 (28h) : Intégration GPS + Tests
- S33 (28h) : Cartographie de champs
- S34 (28h) : Cartographie de champs + Rapport + Soutenance à blanc

Septembre :
- S35 (24h) : Rapport
- S36 (28h) : Tests en extérieurs + Correctifs
- S37 (25h) : Rapport + Correctifs
- S38 (32h) : Tests en extérieurs
- S39 (44h) : Rapport

Octobre :
- S40 (18h) : Rapport + Rendu

Environ : 420h


## Points discutés

- Positionnement -> différentes méthodes
- Meeting avec Sébastien Guillaume prévu le 1er juillet
- Méthode de cartographie de champ
- Plan du cadastre ?
- Soutenance à blanc
  

# Réunion du 1er juillet 2021

## Points discutés

Revues des questions pour le meeting avec Sébastien Guillaume

Questions :
- Utilisation de la photogramétrie pour le positionnement possible ?
- Utilisation de la laserogramètrie pour le positionnement possible ?
- Corrections possibles avec des données atmosphèriques ? ionosphèriques ?
- GPS + INS précision ?
- Meilleure méthode de positionnement pour un robot ?


# Réunion du 7 juillet 2021

## Résumé des points discutés

La validation des coordonnées GPS pourra être validée à l'aide d'un théodolite de la Géomatique. Un théodolite est un appareil de mesure géodésique qui permet d'obtenir des coordonnées au milimètre près.

La distance entre la station de base peut aller jusqu'à 10Km, sachant que plus on est près de la station de base plus on sera précis. La correction est proportionnelle entre la base et le robot. Abonnement Swisstopo possible (RTCM). DGPS offre une position relative.

L'IMU augmente la fréquence de rafraichissement de position en ajoutant des données mais ne donne pas de correction à proprement parler sur la position.

Le problème principal et le multitrajet, pour évité au maximum ces effets, il faut pouvoir régler l'antenne GPS afin d'obtenir un angle plus étroit. Les antennes géodésique permette d'améliorer le multitrajet.

L'utilisation des données L1 et L2 permettent d'annuler l'effet de ka ionosphère. il est également recommandé d'utiliser L1 et L2 si on est loin de la station de base.
Possible de récupérer L1 et utiliser plusieurs stations de bases.
Possible d'utiliser une arduino pour la station de base, voir aussi Adafruit ou Sparkfun.

Une des autres difficultés est la cinématique. le fait que notre robot se déplace constemment ajoute une forte contrainte à la précision positionnement.

Précision au milimètre possible avec surface de réflexion.

Possible d'utiliser la photogramétrie avec des fils au dessus du champs qui guide le robot, position du fil connue, si il ne bouge jamais...

Lidar et piquet, n'offre pas une super précision..

Le robot doit pouvoir recevoir un flux de données xsens (GPS) et du théodolite.

Idéalement il faudra mettre 4 antennes sur le robot avec 4 GPS différent pour faire la moyenne des multitrajet et les comparer pour améliorer la précision..

Si le choix par sur le GPS xsens, pas de réglage possible (blackbox), pas de réglage de cutoff, angle de réception non modifiable, pas le choix des constellations de satelittes.

Déterminer ou mettre le prisme sur le robot, idéalement en dessous de l'antenne GPS et en dessus du reste. Ce prisme permettra de valider les données GPS mesurées.

Meilleure positionnement selon Sébastien Guillaume : Xsens + base + bonne antenne, utiliser RTCM pour la réception via radio des données de la base. Signal radio xbee. Définir comment garder en vie au mieux la station de base. idéalement plusieurs bases, voir si possible avec grange, ferme à proximité...
Partir sur quelque chose qui marche même si pas précis puis cherche comment améliorer la précision.
Etudier comment supprimer au mieux le multitrajet (mousse?)
Probablement ajouter des capteurs pour la detection d'obstacle dans le champ..

Pour les essais devant l'école, Monsieur Guillaume propose d'obtenir une connexion internet pour se connecter aux données mise à disposition par l'école (station de base de l'école) caster ntrip.


# Réunion du 16 juillet 2021

## Points discutés

- Ne pas oublier recherche sur station de base
- Premiers tests avec GPS et PC dès que possible
- Ne pas oublier de mettre le choix final suite à l'analyse
- Moyen de stockage des cartes, gestion des plans/cartes, base de données ?
- Terminer/compléter le planning
- Ajouter les priorités aux problématiques
- Benchmark de la qualité des coordonées GPS
- Mettre le turbo !!


# Réunion du 23 juillet 2021

## Points discutés

- Rapport intermédiaire - remarques et améliorations
- Ajouter le CDC, la clause de confidientialité
- Revoir les problématiques
- Faire photos des champs - contacter Monsieur Bovini
- Fournir nouveau rapport la semaine 34 entre le 27 août et le 29 août


# Réunion du 30 juillet 2021

## Points discutés

- Installer MTManager pour le xsens
- Mettre à jour le firmeware
- Tester le xsens avec MTManager
- Tester et intégrer le xsens dans ROS
- Créer une branche sur robot-reds : tb-martin
- Prendre rdv avec Monsieur Bovigny
- 


# Réunion du 20 août 2021

## Points discutés

- Visite de la ferme du moulin
- Discussion des besoins
- Photos du terrain
- 


# Réunion du 27 août 2021

## Points discutés

- Soutenance à blanc
- Retour sur présentation
- 


# Réunion du 3 septembre 2021

## Points discutés

- Intégrer l'IMU du xsens à la navigation ROS
- Ajouter la théorie de l'IMU au rapport
- Rendre proposition de cahier des charges le 26 septembre
- Apporter amélioration de français/compréhension au rapoprt
- Ajouter les détails des fonctionnalités au rapport
- 


# Réunion du 10 septembre 2021

## Points discutés

- Vérifier les synchronizations xsens
- Définir une stratégie de test
- Détailler les types d'erreurs possibles du GPS
- Prendre nouveau rdv avec Monsieur Guillaume
- 


# Réunion du 17 septembre 2021

## Points discutés

- Trouver nouvelles batteries
- Déterminer la consommation électrique des nouveaux modules
- Tester les variations dans le temps des mesures gnss
- Voir avec Monsieur Guillaume comment fixer l'antenne avec quelque chose de stable
- 


# Réunion du 24 septembre 2021

## Points discutés

- Créer un réseau wifi pour les tests en extérieur avec le robot
- Configurer la station de base
- Tester avec la station de base
- 


# Réunion du 1 octobre 2021

## Points discutés

- Réaliser un schéma pour la communication robot <-> station de base et robot <-> pc
- Réaliser un script pour tests à interval de temps régulier
- Faire copie de l'image de la rpi pour backup
- 


# Réunion du 7 octobre 2021

## Points discutés

- Rendre le résumé le lundi 11.10
- Proposition de date pour la défense
- Avancer sur la rédaction du rapport
- Mettre le code réalisé ou modifié dans le git 2021-tb-martin
- 


# Réunion du 15 octobre 2021

## Points discutés

- Ajouter infos sur l'analyse et le choix dans le résumé pliable
- Rendre nouveau résumé pour lundi 18.10
- Ajouter infos sur l'erreur manuelle et sur la précision/variations du gnss
- (1) Intégration, tests et modifications du gnss à la navigation
- (2) Tests inversés base - robot
- (3) Tests de déplacement et retour (sans robot)
- (4) Tests de déplacement et retour (avec robot)
- 


