	.file	"rvv_fractal.c"
	.option nopic
	.attribute arch, "rv32i2p1_m2p0_f2p2_d2p2_v1p0_zicsr2p0_zmmul1p0_zve32f1p0_zve32x1p0_zve64d1p0_zve64f1p0_zve64x1p0_zvl128b1p0_zvl32b1p0_zvl64b1p0"
	.attribute unaligned_access, 0
	.attribute stack_align, 16
	.text
	.section	.rodata.str1.4,"aMS",@progbits,1
	.align	2
.LC0:
	.string	"computation took : "
	.align	2
.LC1:
	.string	" cycles\n"
	.align	2
.LC2:
	.string	"P2\n"
	.align	2
.LC3:
	.string	" "
	.align	2
.LC4:
	.string	"\n128\n"
	.align	2
.LC5:
	.string	"\n"
	.text
	.align	2
	.globl	rvv_fractal
	.type	rvv_fractal, @function
rvv_fractal:
	addi	sp,sp,-32
	li	t0,-16384
	sw	ra,28(sp)
	sw	s0,24(sp)
	sw	s1,20(sp)
	sw	s2,16(sp)
	sw	s3,12(sp)
	sw	s4,8(sp)
	add	sp,sp,t0
 #APP
# 27 "firmware/rvv_fractal.c" 1
	rdcycle s1;
# 0 "" 2
 #NO_APP
	li	a4,-32
	vsetvli	a5,zero,e32,m1,ta,ma
	vmv.v.x	v14,a4
	li	a4,1048576
	vmv.v.x	v11,a4
	vmv.v.i	v10,0
	mv	s0,sp
	mv	t5,sp
	li	t4,-262144
	li	t6,8192
.L2:
	srai	a5,t4,2
	vmv.v.x	v15,t4
	vmv.v.x	v12,a5
	vid.v	v13
	mv	t3,t5
	li	t1,64
.L4:
	vadd.vv	v1,v13,v14
	li	a3,255
	vmv.v.x	v8,a3
	vsll.vi	v1,v1,14
	vmv1r.v	v2,v15
	vmv.v.i	v5,0
	vsra.vi	v9,v1,2
	vmv.v.i	v4,1
	li	a0,128
	vsetvli	a7,t1,e8,mf4,ta,ma
.L3:
	vsetvli	a4,zero,e32,m1,ta,ma
	vsra.vi	v6,v1,5
	vsra.vi	v3,v2,4
	addi	a0,a0,-1
	vsub.vv	v2,v2,v6
	vadd.vv	v1,v3,v1
	vadd.vv	v2,v2,v12
	vadd.vv	v1,v1,v9
	vsra.vi	v6,v2,31
	vsra.vi	v7,v1,31
	vxor.vv	v0,v2,v6
	vxor.vv	v3,v1,v7
	vsub.vv	v0,v0,v6
	vsub.vv	v3,v3,v7
	vor.vv	v0,v0,v3
	vmsgt.vv	v0,v0,v11
	vmerge.vim	v3,v10,1,v0
	vand.vv	v3,v3,v4
	vmsne.vi	v0,v3,0
	vxor.vi	v3,v3,1
	vmerge.vvm	v8,v8,v5,v0
	vand.vv	v4,v4,v3
	vadd.vi	v5,v5,1
	bne	a0,zero,.L3
	vsetvli	zero,a7,e32,m1,ta,ma
	vse32.v	v8,0(t3)
	vsetvli	a4,zero,e32,m1,ta,ma
	vmv.v.x	v1,a7
	sub	t1,t1,a7
	slli	a7,a7,2
	vadd.vv	v13,v13,v1
	add	t3,t3,a7
	bne	t1,zero,.L4
	li	a5,16384
	addi	t5,t5,256
	add	a5,a5,sp
	add	t4,t4,t6
	bne	t5,a5,.L2
 #APP
# 68 "firmware/rvv_fractal.c" 1
	rdcycle s2;
# 0 "" 2
 #NO_APP
	lui	a0,%hi(.LC0)
	addi	a0,a0,%lo(.LC0)
	call	print_str
	sub	a0,s2,s1
	call	print_dec
	lui	a0,%hi(.LC1)
	addi	a0,a0,%lo(.LC1)
	call	print_str
	lui	a0,%hi(.LC2)
	addi	a0,a0,%lo(.LC2)
	call	print_str
	li	a0,64
	call	print_dec
	lui	s3,%hi(.LC3)
	addi	a0,s3,%lo(.LC3)
	call	print_str
	li	a0,64
	call	print_dec
	lui	a0,%hi(.LC4)
	addi	a0,a0,%lo(.LC4)
	call	print_str
	li	a5,16384
	addi	a5,a5,256
	lui	s4,%hi(.LC5)
	addi	s2,s0,256
	addi	s3,s3,%lo(.LC3)
	add	s0,s0,a5
	addi	s4,s4,%lo(.LC5)
.L6:
	lw	a0,-256(s2)
	addi	s1,s2,-256
	addi	s1,s1,4
	call	print_dec
	mv	a0,s3
	beq	s2,s1,.L7
.L17:
	call	print_str
	lw	a0,0(s1)
	addi	s1,s1,4
	call	print_dec
	mv	a0,s3
	bne	s2,s1,.L17
.L7:
	mv	a0,s4
	addi	s2,s2,256
	call	print_str
	bne	s2,s0,.L6
	li	t0,16384
	add	sp,sp,t0
	lw	ra,28(sp)
	lw	s0,24(sp)
	lw	s1,20(sp)
	lw	s2,16(sp)
	lw	s3,12(sp)
	lw	s4,8(sp)
	addi	sp,sp,32
	jr	ra
	.size	rvv_fractal, .-rvv_fractal
	.ident	"GCC: () 15.1.0"
	.section	.note.GNU-stack,"",@progbits
