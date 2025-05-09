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
- `VLEN >= ELEN` : puissance de 2 ($\le 2^{16} = 64 Kio$), la taille des vecteurs

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

L'exécution d'une instruction vectorielle changeant l'état (incluant les `CSRs`) depuis `mstatus.VS = INITIAL | CLEAN` fait passer `mstatus.VS` à `DIRTY`, et donc `mstatus.SD = 1`, sinon la valeur qui va bien. (`mstatus.SD` indique si une des unités vectorielle/flottante/XS)
