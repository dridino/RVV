# Extension RVV sur base de PicoRV32

## État d'avancement

### Architecture globale

La taille des vecteurs est définie par un paramètre de l'architecture. De même pour le nombre et la largeur des *lanes* de l'ALU vectorielle.

### Accès mémoire

Les lectures et écritures vectorielles fonctionnent avec les modes d'adressage suivants :

- *unit-stride* : accède à la mémoire de manière contigüe
- *strided* : accède à la mémoire suivant un incrément constant (négatif, nul ou positif)
- *unordered-indexed* & *ordered-indexed* : accède à la mémoire à partir d'une adresse de base, à laquelle sont ajoutés des indices contenus dans un (ou plusieurs) vecteurs

### Opérations arithmético-logiques

Les opérations arithmético-logiques de type *vecteur-vecteur* supportées sont :

- `vand`
- `vor`
- `vxor`
- `vadd`

## Simulation

Pour simuler l'exécution de code assembleur riscv sur le processeur, lancer la commande `make test` (ou `make test_vcd` pour générer un fichier `testbench.vcd` visualisable dans gtkwave) depuis le dossier `ressources/picorv32/`. Le fichier contenant le code assembleur est à rentrer dans la constante `TEST_OBJS` du Makefile. La structure des fichiers assembleur diffère des autres afin de pouvoir être exécutés sur le FPGA (sans affichage donc). Si l'exécution ne s'arrête pas, le test est passé, si elle s'arrête, le test n'est pas passé (format nécessaire pour exécution sur FPGA).

## Implémentation sur FPGA

Créer un nouveau projet Quartus avec les fichiers `picorv32/picorv32.v`, `picorv32/vec_alu_wrapper.v`, `picorv32/vec_alu.v` et ceux contenus dans le dossier `quartus_files`, qui contient les descriptions de la mémoire et le fichier *top*.
