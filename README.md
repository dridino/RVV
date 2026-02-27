# Risc-V Vector Extension RTL implementation (RVV)

## Global parameters

The vector size (`VLEN`), the number of lanes (`NB_LANES`) and their width (`LANE_WIDTH`) are parameters of the top module.

## Supported instructions

### Memory instructions

- `vle` / `vse`
- `vluxei` / `vsuxei`
- `vlse` / `vsse`
- `vloxei` / `vsoxei`
- `vlseg` / `vsseg`
- `vl*re` / `vs*r`

### Arithmetic logic instructions

- `vand`
- `vor`
- `vxor`
- `vadd`
- `vsub`
- `vrsub`
- `vmin`
- `vminu`
- `vmax`
- `vmaxu`
- `vsll`
- `vsrl`
- `vsra`
- `vmv`
- `vmerge`

### Mask instructions

Every instruction supporting it can be masked using `v0`.

- `vmand`
- `vmnand`
- `vmandn`
- `vmxor`
- `vmor`
- `vmnor`
- `vmorn`
- `vmxnor`
- `vcpop`
- `vfirst`
- `vmsbf`
- `vmsif`
- `vmsof`
- `viota`
- `vid`
- `vmseq`
- `vmsne`
- `vmsltu`
- `vmslt`
- `vmsleu`
- `vmsle`
- `vmsgtu`
- `vmsgt`

## Simulation

To launch the tests on the PicorRV32, navigate to `ressources/picorv32/` folder and run `make test` (or `make test_vcd` if you want a vcd output). Currently the format of the RVV tests is made to be executed on an FPGA, and therefore the tests run forever. If the test stops, there's an error, if the test run forever or reach a timeout, it passed. This behaviour may change in the future.

## Implementation on an Altera DE10-Lite

To implement the extension and the PicoRV32 on a DE10-Lite, you'll need the following files :

- Top: `quartus_files/top.v`
- Sources: `picorv32/picorv32.v`, `rvv.v`, `rvv_alu_wrapper.v`, `rvv_alu.v`
- Memory initialisation files: `picorv32/firmware/firmware[0-3].mif` files, generated from `firmware.hex` using the `firmware_split.py` script
