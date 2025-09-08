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

Les opérations arithmético-logiques supportées sont :

- `vand`
- `vor`
- `vxor`
- `vadd`

## Simulation


## Implémentation sur FPGA