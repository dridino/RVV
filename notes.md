# Notes quotidiennes

## 09/05/2025

### Verilog

```Verilog
// Module called "dff" has 3 inputs and 1 output port
module dff (input d,
            input clk,
            input rstn,
            output reg q);

    // Contents of the module
    always @ (posedge clk) begin
        if (!rstn)
            q <= 0;
        else
            q <= d;
    end
endmodule

// used to implement shift register
module shift_reg (  input d,
                    input clk,
                    input rstn,
                    output q);

    wire [2:0] q_net;
    dff u0 (.d(d),          .clk(clk), .rstn(rstn), .q(q_net[0]));
    dff u1 (.d(q_net[0]),   .clk(clk), .rstn(rstn), .q(q_net[1]));
    dff u2 (.d(q_net[1]),   .clk(clk), .rstn(rstn), .q(q_net[2]));
    dff u3 (.d(q_net[2]),   .clk(clk), .rstn(rstn), .q(q));

endmodule
```

#### Déclaration de ports

Par défaut `wire` : connection directe entre deux éléments, assignation continue
On peut aussi spécifier `reg` qui permet d'enregistrer la valeur. Valable uniquement pour des ports de sortie (`output`) et utilisable seulement dans des blocks procéduraux `always` et `initial`.

#### Opérateurs

- `assign` : affectation **immédiate** d'une variable de type `wire`, peut contenir une expression dépendant de plusieurs signaux. Ne convient pas à la MAJ d'une variable de type `reg`.
- `+`, `-`, `*`, `/`, `%`, `**`
- `<`, `>`, `<=`, `>=`
- `===` (eq incluant `X` et `Z`), `!==` (neq incluant `X` et `Z`), `==` (eq résultat p-e inconnu), `!=` (neq résultat p-e inconnu),
- `&&`, `&`, `||`, `|`, `!` convertit une valeur !=0 en valeur =0 et vice-versa
- `<<` (lsl), `<<<` asl, `>>` (rsl), `>>>` (asr)
- concaténation : `{elem1[1:0], elem2, ..., elemN};`
- réplication : `assign res = {7{a}};` (7 ne peut pas être une variable, ne pas utiliser pour les signaux `output` / `inout`);

#### `always` blocks

```Verilog
always @ (event)
    [statement]

always @ (event) begin
    [multiple statements]
end
```

### Implémentation du PICO-RV32

> Les 32 registres sont stockés dans la variable `cpuregs`, au même titre que les registres gérant les `IRQs`.

### RVV

#### ISA

<https://storage.googleapis.com/shodan-public-artifacts/RVV-Specification-Docs/riscv-v-spec-1.0-frozen-for-public-review.pdf>

#### Paramètres

- `ELEN >= 8` : puissance de 2, la taille max des éléments des vecteurs
- `VLEN >= ELEN` : puissance de 2 ($\le 2^{16} = 64$ Kib), la taille des vecteurs

#### Registres

- 32 registres vectoriels (`v0` - `v31`) de longueurs fixes `VLEN`.
- 7 "unprivileged" CSRs :

|Adress|Privilege|Name|Description|
|---|---|---|---|
|0x008|URW|`vstart`|Vector start position|
|0x009|URW|`vxsat`|Fixed-Point Saturate Flag|
|0x00A|URW|`vxrm`|Fixed-Point Rounding Mode|
|0x00F|URW|`vcsr`|Vector control and status register|
|0x020|URO|`vl`|Vector length|
|0x021|URO|`vtype`|Vector data type register|
|0x022|URO|`vlenb`|`VLEN/8` (vector register length in bytes)|

`mstatus` register :

![mstatus register](<https://five-embeddev.com/riscv-priv-isa-manual/Priv-v1.12/machine_05.svg>)

Signification du champ `VS` dans le registre `mstatus` :

- `0` : **OFF**, pas implémenté, erreur si tentative d'utilisation
- `1` : **INITIAL**, implémenté, pas d'opération vectorielle réalisée depuis le dernier reset
- `2` : **CLEAN**, implémenté, l'unité vectorielle a été init ou restaurée, pas d'opération vectorielle depuis
- `3` : **DIRTY**, implémenté, l'état de l'unité vectorielle a subit des modifications depuis le dernier reset / restauration

L'exécution d'une instruction vectorielle changeant l'état (incluant les `CSRs`) depuis `mstatus.VS = INITIAL | CLEAN` fait passer `mstatus.VS` à `DIRTY`, et donc `mstatus.SD = 1`, sinon la valeur qui va bien. (`mstatus.SD` indique si une des unités vectorielle/flottante/XS est `DIRTY`)

## 12/05/2025

### RVV

#### Registre `vtype`

Sur `XLEN = 32` bits, lecture seule, décrit comment doit être interprété le contenu des registres vecteurs et comment doit être traité les valeurs débordantes.

- `(XLEN-1)` : `vill`, à 1 indique que la configuration demandée n'est pas supportée
- `(XLEN-2 : 8)` : `0`, reservé si différent de 0
- `(7)` : `vma`, *vector mask agnostic*
- `(6)` : `vta`, *vector tail agnostic*
- `(5 : 3)` : `vsew(2:0)`, largeur des éléments (*SEW = selected element width*). (`0b000=8`, `0b001=16`, `0b010=32`, `0b011=64`, `0b1XX=réservé`)
- `(2 : 0)` : `vlmul(2:0)`, nombre de vecteurs à considérer pour cette instruction. Peut être entier ou fractionnaire (toujours puissance de 2)

##### LMUL

La valeur de `VLMAX = LMUL*VLEN/SEW` représente le nombre max d'éléments qui peuvent être opéré par une seule instruction, comme indiqué dans la table suivante :

![LMUL tableau](./ressources/md_ressources/lmul_tab.png)

> Le numéro du registre de base `v` doit être multiple de `LMUL` (pour `LMUL > 1`), sinon c'est invalide

##### VTA / VMA

Les éléments de destination `tail` et `inactive` sont les éléments d'un registre qui ne reçoivent pas de valeur lors d'une instruction

![VTA/VMA tableau](./ressources/md_ressources/vta_vma.png)

> Quoiqu'il arrive, les éléments masqués de la *queue* sont toujours traîtés comme *agnostique*, indépendamment de la valeur de `vta`.

> `undisturbed` : la valeur contenue dans les éléments concernés ne change pas

> `agnostic` : soit la valeur contenue dans les éléments concernés est conservée (comme `undistrubed`), soit elle est remplacée par des `1`, avec une combinaison aléatoire et pas nécessairement déterministe

Configuration du registre `vtype` :

```mips
    ta   # Tail agnostic
    tu   # Tail undisturbed
    ma   # Mask agnostic
    mu   # Mask undisturbed

    vsetvli t0, a0, e32, m4, ta, ma   # Tail agnostic, mask agnostic
    vsetvli t0, a0, e32, m4, tu, ma   # Tail undisturbed, mask agnostic
    vsetvli t0, a0, e32, m4, ta, mu   # Tail agnostic, mask undisturbed
    vsetvli t0, a0, e32, m4, tu, mu   # Tail undisturbed, mask undisturbed
```

##### vill

Permet d'encoder que l'appel précédent à `vset{i}vl{i}` a tenté d'écrire une valeur non supportée dans `vtype`. Cela permet de brancher sur le bit de poids fort de `vtype` afin de savoir si on doit lever une exception ou non.

Tous les bits de `vtype` doivent être vérifiés pour définir la valeur de `vill` et déterminer ou non si c'est compatible avec l'architecture plutôt que d'exécuter quelque chose d'erroné.

Si `vill = 1`, toute tentative d'exécution d'instruction dépendant de `vtype` lèvera une `IllegalInstructionException`. Cela ne concerne donc pas les instructions `vset{i}vl{i}` et les lectures/écritures/moves de vecteurs complets.

Si `vill = 1`, tous les autres bits de `vtype` doivent être mis à `0`.

#### Registre `vl` : *Vector Length*

Registre de `XLEN = 32` bits, lecture seule.

Le nombre d'éléments (`unsigned`) à MAJ avec le résultat d'une instruction vectorielle. Ne peut être modifié que par `vset{i}vl{i}` ou des *"fault-only-first vector load instruction variants"*. Ce nombre prend en compte la valeur de `LMUL`, i.e. avec `VLEN=32`, `LMUL=8`, `SEW=8` on a `VL=32` (4 éléments par vecteur, fois 8 vecteurs).

#### Registre `vlenb` : *Vector Byte Length*

Registre de `XLEN = 32` bits, lecture seule. Rien à voir avec `vl` juste au dessus.

Vaut `VLEN/8`. Ca doit être une constante dans le design.

#### Registre `vstart` : *Vector Start Index CSR*

Spécifie l'indice du premier élément à être exécuté par une instruction vectorielle (cf 3.7)

> `vstart` est remis à `0` à la fin de l'exécution de chaque instruction vectorielle, même `vset{i}vl{i}`.

> `vstart` n'est pas modifié par une instruction qui lève une `IllegalInstructionException`

> `vstart` doit avoir suffisamment de bits (pas plus) pour pouvoir indicer tous les éléments avec `LMUL` maximal et `SEW` minimal

> `vstart` ne doit pas contenir de valeur plus grande que l'indice max d'un élément, recommandé de *trap* dans ces cas là

> Normalement, `vstart` est seulement écrit par le matériel. Il peut être écrit par du *unprivileged code* mais c'est peu recommandé pour les applications et peut causer de forts ralentissements sur certaines implémentations. Néanmoins nécessaire pour des libs de *threading* au niveau utilisateur.

> L'implémentation peut lever une `IllegalInstructionException` si la valeur de `vstart` n'est pas obtenable via une exécution à partir de `vstart=0`. Exemple : certaines implémentations n'interrompent pas les instructions vectorielles arithmétiques et traitent les IRQ après. Ainsi, quand l'implémentation exécute une instruction avec `vstart!=0` elle peut lever une exception

#### Registre `vxrm` : *Vector fixed-point rounding mode*

Registre **R/W** de longueur `XLEN` dont les 2 LSB détermine la méthode d'arrondi. Les autres sont mis à `0`. En notant `v` la valeur pré-arrondi et `d` le nombre de bits à arrondir on a :

![vxrm tableau](./ressources/md_ressources/vxrm.png)

Les fonctions d'arrondi sont :

```c
roundoff_unsigned(v,d) = (unsigned(v) >> d) + r
roundoff_signed(v,d) = (signed(v) >> d) + r
```

#### Registre `vxsat` : *Vector fixed-point saturation flag*

Registre **R/W** dont le LSB indique si une instruction *fixed-point* a dû saturer une sortie pour rentrer dans le format de destination. Les autres bits doivent être mis à `0`.

Ce bit est cloné dans le registre `vcsr`.

#### Registre `vcsr` : *Vector Control and Status*

Les deux valeurs des registres précédents (`vxrm`, `vxsat`) peuvent aussi être accédées depuis `vcsr`.

- `vcsr(2:1) = vxrm(1:0)`
- `vcsr(0) = vxsat(0)`

#### État de l'extension vectorielle au reset

> `vtype` et `vl` doivent avoir des valeurs qui peuvent être lues et restaurées avec une seule instruction `vsetvl`. Il est recommandé de mettre `vtype.vill = 1` et le reste de `vtype` à 0, `vl` à `0`

> `vstart`, `vxrm`, `vxsat` peuvent prendre des valeurs arbitraires au reset (car `vstart` recquiert quoiqu'il arrive une instruction `vset{i}vl{i}` et (`vxrm`, `vxsat`) doivent être initialisés clairement par le SW)

> Les registres vectoriels peuvent avoir des valeurs arbitraires au reset

### Verilog

#### *Reduction operators*

```Verilog
    wire [3:0] a = 4'b1010;

    wire result_and   = &a;   // 1 & 0 & 1 & 0 = 0
    wire result_or    = |a;   // 1 | 0 | 1 | 0 = 1
    wire result_xor   = ^a;   // 1 ^ 0 ^ 1 ^ 0 = 0
    wire result_nand  = ~&a;  // ~(1 & 0 & 1 & 0) = 1
    wire result_nor   = ~|a;  // ~(1 | 0 | 1 | 0) = 0
    wire result_xnor1 = ~^a;  // ~(1 ^ 0 ^ 1 ^ 0) = 1
    wire result_xnor2 = ^~a;  // Same as above
```

## 13/05/2025

### Implémentation `vadd.vv`

#### Idée générale

```mips
vadd.vv vd, vs2, vs1, vm # vector-vector add
```

```c
for (int i = 0; i < ((VLEN*LMUL)/SEW) - 1; i++) {
    res_17 = vs1[SEW*(i+1) : SEW*i] + vs2[SEW*(i+1) : SEW*i];
    vd[SEW*(i+1) : SEW*i] = res_17[SEW-1 : 0];
}
```

## 14/05

### PicoRV32 - Simulation

Pour générer un fichier `ram.hex` contenant le programme, exécuter `make all` depuis le dossier souhaité dans le répertoire `software` (depuis wsl)

Pour générer un fichier `bootloader.hex`, exécuter `make all` depuis le répertoire `bootloader/bootloader/` (depuis wsl)

Pour lancer la simulation (depuis windows) :

- `cd DE1/sim_work`
- `vlog  +acc -l comp.log -timescale "1ns/1ps" +define+IDEBUG +define+SIMULATION +incdir+../includes -f ../comp_file`
- `vsim -L altera_mf soc_core_tb`

#### Executer de l'assembleur

Obligé de mettre les instructions à la main au début du bootloader ?

## 16/05

### PicoRV32

<https://fprox.substack.com/p/how-to-read-risc-v-vector-assembly>

## 19/05

### PicoRV32

Compilation de tests persos impossibles pour des instructions vectorielles 32 bits, la *toolchain* le supporte pas RVV. Voir une autre *toolchain* ?

#### Implémentation RVV

- Toujours 32 registres vectoriels ou utiliser la constante `ENABLE_REGS_16_31` ?
- Le registre `mstatus` est présent dans l'architecture ?
- supporter les instructions vectorielles flottantes sachant que les scalaires ne le sont pas ? -> pas de registre flottant par conséquent

## 22/05

### PICORV32

> L'instruction `vset{i}vl{i}` s'exécute correctement, reste à voir les cas limites

## 02/06

### PICORV32

> L'instruction `vset{i}vl{i}` s'exécute parfaitement.

## 04/06

Fonctionnement de l'instruction `vload` :

|SEW\LMUL|1/8|1/4|1/2|1|2|4|8|
|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
|8b| x | x | x | x | x | x | x |
|16b| x | x | x | x | x | x | x |
|32b| - | x | x | x | x | x | x |
|64b| - | - | x | x | x | x | x |

Les `~vl` accès mémoire sont réalisés puis la prochaine instruction est rechargée en envoyant pc à la mémoire. Pas le même système que les accès scalaire car ils ont un état en plus.

## 05/06

> Plusieurs `vload` à la suite fonctionnent.

> `vstart` *unit-stride* fonctionne

## 09/06

> Changement de la manière dont est chargée l'instruction après un accès mémoire vectoriel : on la met dans un registre intermédiaire pendant l'opération et la remet dans `mem_rdata_q` plutôt que refaire un accès mémoire à la fin. Cette dernière étant déjà dans `rdata` au moment de faire les accès mémoires de données

> `vload` | `vstore` fonctionnels avec mode d'adressage `unit-stride` et `strided`. Le mode d'adressage `unit-stride` ne groupe pas les accès mémoire, conformément à la doc.

## 11/06

> Pour le mode d'adressage `strided`, une valeur positive, négative ou nulle fonctionne

> Mode d'adressage `indexed` fonctionne

## 12/06

### traps

> Gestion des traps et émission de traps selon les conditions ci-dessous.

Conditions pour trap :

- bit `vill=1` et on veut exécuter une instruction autre que `vset{i}vl{i}` ou *whole-register load/store/move*
- quand vstart est *out-of-bounds*, recommandé de trap mais pas obligé
- quand la valeur des registres spécifiée n'est pas compatible avec vtype. Exemple : v31 & LMUL=2

> Encodage de vstart dans le cadre de transferts mémoire (différents des transferts de masques qui sont des cas particuleirs) : `vstart = {00, indexed_reg_i, indexed_byte_i, reg_i, byte_i}`

										$display("byte0");
## 13/06

### Segment load / store

> Idée : un indice permettant de savoir à quel registre (*field*) on en est.

## 16/06

Test des différentes variantes de `vlseg{seg}e{sew}.v` :

|SEW\SEG|2|3|4|5|6|7|8|
|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
|8b| (M1) | (M1) |  |  |  |  |  |
|16b| (M1,M2,M4) | (M1) |  | (MF2) |  |  | (MF2,M1,ERR) |
|32b| (M1) | (M1) |  |  |  |  |  |
|64b| (M1) | (M1) |  |  |  | (M1) |  |

> segment load & store fonctionnent parfaitement

> whole register load & store fonctionnent aussi

## 18/06

> vand.vv fonctionne dans l'alu vectorielle

## 19/06

> ALU vectorielle paramétrable (nombre et largeur des *lanes*)

## 23/06

> Le nombre de *lanes* est maintenant un argument et non un paramètre de l'ALU, permettant de tenir compte des cas où on a trop d'ALU par rapport au besoin (ex : VLEN=128, SEW=64 et NB_LANES=4, on a un potentiel de 4x64=256 bits mais seul 128 sont utilisés)

> Modèle fonctionne avec toutes les combinaisons (largeur x nb) : `(8b,16b,32b,64b) x (1,2,4)`

## 24/06

> L'ALU est maintenant synthétisable

## 25/06

> ALU marche avec plusieurs lanes en passant par un wrapper

> Comparaison des ressources utilisées par les différentes méthodes de padding des éléments dans les ALU

```
Padding à 0

4.26. Printing statistics.

=== $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'000 ===

   Number of wires:               2381
   Number of wire bits:           2788
   Number of public wires:          18
   Number of public wire bits:     413
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:               2419
     $_ANDNOT_                     530
     $_AND_                         27
     $_AOI3_                       156
     $_DFF_P_                       16
     $_MUX_                        549
     $_NAND_                       183
     $_NOR_                         31
     $_NOT_                         93
     $_OAI3_                       261
     $_OAI4_                        15
     $_ORNOT_                       63
     $_OR_                         331
     $_XNOR_                        36
     $_XOR_                        128

=== $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'001 ===

   Number of wires:               2277
   Number of wire bits:           2732
   Number of public wires:          18
   Number of public wire bits:     461
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:               2315
     $_ANDNOT_                     417
     $_AND_                         28
     $_AOI3_                        97
     $_DFF_P_                       16
     $_MUX_                        622
     $_NAND_                       175
     $_NOR_                         46
     $_NOT_                         77
     $_OAI3_                       238
     $_OAI4_                        19
     $_ORNOT_                      115
     $_OR_                         326
     $_XNOR_                        32
     $_XOR_                        107

=== $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'010 ===

   Number of wires:               2396
   Number of wire bits:           2803
   Number of public wires:          18
   Number of public wire bits:     413
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:               2434
     $_ANDNOT_                     538
     $_AND_                         27
     $_AOI3_                       157
     $_DFF_P_                       16
     $_MUX_                        548
     $_NAND_                       179
     $_NOR_                         29
     $_NOT_                         94
     $_OAI3_                       262
     $_OAI4_                        15
     $_ORNOT_                       65
     $_OR_                         333
     $_XNOR_                        36
     $_XOR_                        135

=== $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'011 ===

   Number of wires:               2395
   Number of wire bits:           2802
   Number of public wires:          18
   Number of public wire bits:     413
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:               2433
     $_ANDNOT_                     530
     $_AND_                         24
     $_AOI3_                       157
     $_DFF_P_                       16
     $_MUX_                        548
     $_NAND_                       185
     $_NOR_                         33
     $_NOT_                         94
     $_OAI3_                       264
     $_OAI4_                        15
     $_ORNOT_                       64
     $_OR_                         331
     $_XNOR_                        38
     $_XOR_                        134

=== vec_alu_wrapper ===

   Number of wires:                 24
   Number of wire bits:            576
   Number of public wires:          24
   Number of public wire bits:     576
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:                  4
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'000      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'001      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'010      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'011      1

=== design hierarchy ===

   vec_alu_wrapper                   1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'000      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'001      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'010      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'011      1

   Number of wires:               9473
   Number of wire bits:          11701
   Number of public wires:          96
   Number of public wire bits:    2276
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:               9601
     $_ANDNOT_                    2015
     $_AND_                        106
     $_AOI3_                       567
     $_DFF_P_                       64
     $_MUX_                       2267
     $_NAND_                       722
     $_NOR_                        139
     $_NOT_                        358
     $_OAI3_                      1025
     $_OAI4_                        64
     $_ORNOT_                      307
     $_OR_                        1321
     $_XNOR_                       142
     $_XOR_                        504
```

```
Padding avec le dernier bit

4.26. Printing statistics.

=== $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'000 ===

   Number of wires:               3166
   Number of wire bits:           3573
   Number of public wires:          18
   Number of public wire bits:     413
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:               3204
     $_ANDNOT_                     633
     $_AND_                         47
     $_AOI3_                        86
     $_AOI4_                         8
     $_DFF_P_                       16
     $_MUX_                        740
     $_NAND_                        53
     $_NOR_                         26
     $_NOT_                        324
     $_OAI3_                       168
     $_OAI4_                        15
     $_ORNOT_                       69
     $_OR_                         848
     $_XNOR_                        37
     $_XOR_                        134

=== $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'001 ===

   Number of wires:               2958
   Number of wire bits:           3413
   Number of public wires:          18
   Number of public wire bits:     461
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:               2996
     $_ANDNOT_                     163
     $_AND_                         28
     $_AOI3_                        63
     $_DFF_P_                       16
     $_MUX_                        770
     $_NAND_                        52
     $_NOR_                         42
     $_NOT_                        321
     $_OAI3_                        86
     $_OAI4_                        31
     $_ORNOT_                       63
     $_OR_                        1215
     $_XNOR_                        31
     $_XOR_                        115

=== $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'010 ===

   Number of wires:               3430
   Number of wire bits:           3837
   Number of public wires:          18
   Number of public wire bits:     413
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:               3468
     $_ANDNOT_                     626
     $_AND_                         29
     $_AOI3_                       112
     $_AOI4_                         8
     $_DFF_P_                       16
     $_MUX_                        869
     $_NAND_                        51
     $_NOR_                         35
     $_NOT_                        329
     $_OAI3_                       147
     $_OAI4_                        15
     $_ORNOT_                       58
     $_OR_                         996
     $_XNOR_                        41
     $_XOR_                        136

=== $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'011 ===

   Number of wires:               3426
   Number of wire bits:           3833
   Number of public wires:          18
   Number of public wire bits:     413
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:               3464
     $_ANDNOT_                     628
     $_AND_                         34
     $_AOI3_                       115
     $_AOI4_                         8
     $_DFF_P_                       16
     $_MUX_                        864
     $_NAND_                        50
     $_NOR_                         35
     $_NOT_                        329
     $_OAI3_                       148
     $_OAI4_                        15
     $_ORNOT_                       59
     $_OR_                         985
     $_XNOR_                        42
     $_XOR_                        136

=== vec_alu_wrapper ===

   Number of wires:                 24
   Number of wire bits:            576
   Number of public wires:          24
   Number of public wire bits:     576
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:                  4
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'000      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'001      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'010      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'011      1

=== design hierarchy ===

   vec_alu_wrapper                   1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'000      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'001      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'010      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'011      1

   Number of wires:              13004
   Number of wire bits:          15232
   Number of public wires:          96
   Number of public wire bits:    2276
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:              13132
     $_ANDNOT_                    2050
     $_AND_                        138
     $_AOI3_                       376
     $_AOI4_                        24
     $_DFF_P_                       64
     $_MUX_                       3243
     $_NAND_                       206
     $_NOR_                        138
     $_NOT_                       1303
     $_OAI3_                       549
     $_OAI4_                        76
     $_ORNOT_                      249
     $_OR_                        4044
     $_XNOR_                       151
     $_XOR_                        521
```

```
Pas de padding

4.26. Printing statistics.

=== $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'000 ===

   Number of wires:               2097
   Number of wire bits:           2504
   Number of public wires:          18
   Number of public wire bits:     413
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:               2135
     $_ANDNOT_                     332
     $_AND_                         25
     $_AOI3_                        56
     $_AOI4_                         1
     $_DFF_P_                       16
     $_MUX_                        521
     $_NAND_                       157
     $_NOR_                         51
     $_NOT_                         80
     $_OAI3_                       233
     $_OAI4_                        15
     $_ORNOT_                      102
     $_OR_                         357
     $_XNOR_                        42
     $_XOR_                        147

=== $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'001 ===

   Number of wires:               2119
   Number of wire bits:           2574
   Number of public wires:          18
   Number of public wire bits:     461
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:               2157
     $_ANDNOT_                     349
     $_AND_                         28
     $_AOI3_                        54
     $_AOI4_                         1
     $_DFF_P_                       16
     $_MUX_                        516
     $_NAND_                       157
     $_NOR_                         35
     $_NOT_                         83
     $_OAI3_                       204
     $_OAI4_                        15
     $_ORNOT_                      102
     $_OR_                         398
     $_XNOR_                        42
     $_XOR_                        157

=== $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'010 ===

   Number of wires:               2124
   Number of wire bits:           2579
   Number of public wires:          18
   Number of public wire bits:     461
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:               2162
     $_ANDNOT_                     334
     $_AND_                         19
     $_AOI3_                        51
     $_AOI4_                         1
     $_DFF_P_                       16
     $_MUX_                        520
     $_NAND_                       148
     $_NOR_                         46
     $_NOT_                         94
     $_OAI3_                       231
     $_OAI4_                        15
     $_ORNOT_                      101
     $_OR_                         390
     $_XNOR_                        44
     $_XOR_                        152

=== $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'011 ===

   Number of wires:               2104
   Number of wire bits:           2511
   Number of public wires:          18
   Number of public wire bits:     413
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:               2142
     $_ANDNOT_                     352
     $_AND_                         25
     $_AOI3_                        47
     $_AOI4_                         1
     $_DFF_P_                       16
     $_MUX_                        520
     $_NAND_                       147
     $_NOR_                         39
     $_NOT_                         94
     $_OAI3_                       220
     $_OAI4_                        15
     $_ORNOT_                       88
     $_OR_                         380
     $_XNOR_                        38
     $_XOR_                        160

=== vec_alu_wrapper ===

   Number of wires:                 24
   Number of wire bits:            576
   Number of public wires:          24
   Number of public wire bits:     576
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:                  4
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'000      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'001      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'010      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'011      1

=== design hierarchy ===

   vec_alu_wrapper                   1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'000      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'001      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'010      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'011      1

   Number of wires:               8468
   Number of wire bits:          10744
   Number of public wires:          96
   Number of public wire bits:    2324
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:               8596
     $_ANDNOT_                    1367
     $_AND_                         97
     $_AOI3_                       208
     $_AOI4_                         4
     $_DFF_P_                       64
     $_MUX_                       2077
     $_NAND_                       609
     $_NOR_                        171
     $_NOT_                        351
     $_OAI3_                       888
     $_OAI4_                        60
     $_ORNOT_                      393
     $_OR_                        1525
     $_XNOR_                       166
     $_XOR_                        616
```

impact sur le matériel de la routine ajoutant le *padding*
- lourd (données par ALU) :
    - *padding* avec des `0` : +300 (+14%) *cells*, +300 (+12%) *wire bits*
    - *padding* avec le dernier bit : +1100 (+52%) *cells*, +1100 (+44%) *wire bits*
- de plus, ça ne sert à rien

Conclusion : ça fonctionne sans *padding* en post-synthèse et ça génère moins de matériel, autant garder ça.

## 26/06

Tentative d'envoyer que des vecteurs de 64b du wrapper aux ALU contre `VLEN` bits auparavant. Résulats :

- (-61%) de *wire bits* (750 vs 1900) et (-68%) de *cells* (480 vs 1500) par ALU
- (+527%) de *wire bits* (9400 vs 1500) et (+833%) de *cells* (8823 vs 945) pour le wrapper
- (+33%) de *wire bits* (12350 vs 9300) et (+50%) de *cells* (10700 vs 7150) pour une architecture à 4 ALU

Cette architecture serait rentable que pour un nombre **très grand** d'ALU. Ainsi, Elle ne sera pas retenue et on continuera de passer les 2x128b de données.

> Le calcul d'indices a été déporté dans le wrapper, La répartition de quelle *lane* doit tourner est aussi faite dans le wrapper plutôt que dans le TB (et donc le module RVV), des signaux ont été ajoutés pour dire au module RVV quelle valeur a été MAJ

## 27/06

> Intégration de l'ALU dans le pico, supporte différents LMUL et vl (même n'occupant pas les vecteurs en entier)
> L'architecture va jusqu'à 8 *lanes*

## 29/06

> Synthèse des ALUs, marche nickel
> Test de plusieurs instructions arithmétiques vectorielles à la suite : OK
> pb en post-synth : plusieurs drivers pour `mem_rdata_q`, `mem_wstrb`, `mem_wdata`, `mem_addr`, `mem_instr`, `mem_valid` et `trap`

## 30/06

> Synthèse de tout le proc avec RVV fonctionne (validé par simu post-synthèse)

## 03/07

> Le processeur avec extension RVV tourne sur la DE10-Lite : sans module de multiplication ni division on peut mettre 2 lanes de 32 bits avec des vecteurs de 128 bits
> 4 lanes de 16 bits ne passe pas à cause du placement routage (45+ minutes, toujours pas fini)

## 14/09

> Le processeur tourne sur la DE10-Lite avec le même code que celui de simu. Les résultats peuvent varier en fonction des signaux observés via le *SignalTap Analyze* (peut-être un problème de synthèse des registres vectoriels ?)
> A la simulation comportementale, l'ALU génère des `x` car on lit en dehors du vecteur, comportement disparaissant en synthèse et ne posant aucun problème dans ce cadre. Cela se produit quand le `vsew` est plus petit que la largeur des *lanes* de notre architecture.









## Réponses

- regarder vcd pour l'apparition de `x` quand on déborde
    - disparaît en post-synthèse
- impact sur le matériel de la routine ajoutant le *padding*
    - lourd (données par ALU) :
        - *padding* avec des `0` : +300 (+14%) *cells*, +300 (+12%) *wire bits*
        - *padding* avec le dernier bit : +1100 (+52%) *cells*, +1100 (+44%) *wire bits*
    - de plus, ça ne sert à rien


## Questions

> L'architecture fonctionne en simu post-synthèse mais plus en simu comportementale (sûrement un problème de *timings*), on est d'accord que c'est OK ?

> Deux versions du wrapper, une avec 1 seul signal indiquant la terminaison, une autre avec 1 signal par ALU. Celle avec un seul signal utilise plus de fils et cells que celle avec plusieurs, chelou ? Rattrapé par l'utilisation côté module RVV ?

```
Plusieurs signaux done
=== vec_alu_wrapper ===

   Number of wires:                910
   Number of wire bits:           1522
   Number of public wires:          34
   Number of public wire bits:     634
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:                945
     $_ANDNOT_                     173
     $_AND_                         31
     $_AOI3_                        63
     $_DFF_P_                       15
     $_MUX_                        129
     $_NAND_                        25
     $_NOR_                         53
     $_NOT_                         67
     $_OAI3_                        40
     $_ORNOT_                       61
     $_OR_                         105
     $_XNOR_                        33
     $_XOR_                        146
```

```
Un seul signal done
=== vec_alu_wrapper ===

   Number of wires:               1021
   Number of wire bits:           1633
   Number of public wires:          28
   Number of public wire bits:     628
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:               1062
     $_ANDNOT_                     190
     $_AND_                         22
     $_AOI3_                        54
     $_AOI4_                         1
     $_DFF_P_                       15
     $_MUX_                        131
     $_NAND_                        37
     $_NOR_                         78
     $_NOT_                         69
     $_OAI3_                        49
     $_ORNOT_                       69
     $_OR_                         137
     $_XNOR_                        38
     $_XOR_                        168
```

- Quand je tronque le résultat de l'addition entre la retenue et un élément pour garder des additionneurs de même taille (largeur de la lane) plutôt qu'avoir un de (largeur de lane) et un 1 bit plus large, ça rajoute 100 *cells* et *wire bits* par ALU. Je le fais quand-même ou non ?

- truc bizarre quand on utilise moins de 8 ALUs :
  - 1 seul gros vecteur en sortie de wrapper dans lequel chaque ALU écrit **directement** : introduction de `x` dans les bits de poids faibles de la première ALU.
  - `NB_LANES` vecteurs dans lesquels chaque ALU écrit (dans le vec correspondant) tous concaténés dans la grosse sortie : fonctionne nickel

## Améliorations

### Non-implémenté

Conditions pour raise `Illegal Instruction` :

- quand on se retrouve avec un vstart que le programme n'aurait jamais pu produire avec ce vtype

- Fault-only-first load & store : pas d'info venant de la mémoire pour déterminer si l'accès valide ou non
- Paramètre pour dire si on trap ou non quand les registres ne sont pas alignés sur `EMUL`
- Mettre une cause précise lors d'un trap venant de RVV (je trouve pas de registre de cause même dans le coeur classique du pico)
- Gérer toutes les raisons de trap
- Bonne gestion de vstart
- Load & Store de masques (qui vont uniquement dans `v0`)

### Optionnel

- *pipeliner* les accès mémoire