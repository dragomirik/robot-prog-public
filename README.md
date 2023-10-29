# robot-prog
Controle tout sauf l'IA, code du microcontroleur C du robot

Choix final de programmer avec l'IDE Arduino pour éviter des problèmes de compilation avec les makefile custom
Pour l'instant tout le code est dans un seul fichier .ino mais il pourrait potentiellement être divisé de deux façons différentes :
* Soit en différents fichiers .ino, utilisant des namespaces pour contourner la concaténation par ordre alphabétique et contenant donc dans le même fichier la déclaration et l'implémentation
  * Avantages : sur que cela fonctionne
  * Inconvénients : mauvaise lisibilité (mais meilleure que lorsque tout est dan sun seul fichier)
* Soit directement en .hpp et .cpp
  * Avantages : clair simple précis et utilise les fonctionnalités completes c++
  * Inconvénients : compatibilité à tester notamment l'accés aux fonctions natives Arduino définies dans `Arduino.h`
