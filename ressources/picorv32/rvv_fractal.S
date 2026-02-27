	.file	"rvv_fractal.c"
	.option nopic
	.attribute arch, "rv32i2p1_m2p0_zmmul1p0"
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
	addi	sp,sp,-48
	sw	ra,44(sp)
	sw	s0,40(sp)
	sw	s1,36(sp)
	sw	s2,32(sp)
	sw	s3,28(sp)
	sw	s4,24(sp)
	sw	s5,20(sp)
	sw	s6,16(sp)
	sw	s7,12(sp)
	sw	s8,8(sp)
	addi	sp,sp,-2048
	addi	sp,sp,-2048
 #APP
# 27 "firmware/rvv_fractal.c" 1
	rdcycle s4;
# 0 "" 2
 #NO_APP
	li	a7,1048576
	mv	s3,sp
	addi	a7,a7,1
	li	s2,0
	li	t0,-262144
	li	t1,128
	li	s0,16384
	li	t2,524288
	li	s6,8192
	li	s5,4096
.L2:
	srai	t3,t0,2
	mv	t6,s3
	li	t5,-524288
.L5:
	mv	a4,t5
	srai	a6,t5,2
	mv	a3,t0
	li	a0,1
	li	t4,255
	li	a1,0
.L4:
	srai	a2,a4,5
	srai	a5,a3,4
	add	a4,a5,a4
	sub	a3,a3,a2
	add	a3,a3,t3
	add	a4,a4,a6
	srai	s8,a3,31
	srai	s7,a4,31
	xor	a5,s8,a3
	xor	a2,s7,a4
	sub	a2,a2,s7
	sub	a5,a5,s8
	or	a5,a5,a2
	slt	a5,a5,a7
	seqz	a5,a5
	and	a5,a5,a0
	addi	a2,a1,1
	beq	a5,zero,.L3
	mv	t4,a1
	li	a0,0
.L3:
	andi	a1,a2,0xff
	bne	a1,t1,.L4
	sb	t4,0(t6)
	add	t5,t5,s0
	addi	t6,t6,1
	bne	t5,t2,.L5
	addi	s2,s2,64
	add	t0,t0,s6
	addi	s3,s3,64
	bne	s2,s5,.L2
 #APP
# 68 "firmware/rvv_fractal.c" 1
	rdcycle s0;
# 0 "" 2
 #NO_APP
	lui	a0,%hi(.LC0)
	addi	a0,a0,%lo(.LC0)
	call	print_str
	sub	a0,s0,s4
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
	li	a5,4096
	addi	a5,a5,64
	lui	s4,%hi(.LC5)
	addi	s3,s3,%lo(.LC3)
	addi	s2,sp,64
	add	s1,sp,a5
	addi	s4,s4,%lo(.LC5)
.L7:
	lbu	a0,-64(s2)
	addi	s0,s2,-64
	addi	s0,s0,1
	call	print_dec
	mv	a0,s3
	beq	s2,s0,.L8
.L16:
	call	print_str
	lbu	a0,0(s0)
	addi	s0,s0,1
	call	print_dec
	mv	a0,s3
	bne	s2,s0,.L16
.L8:
	mv	a0,s4
	addi	s2,s2,64
	call	print_str
	bne	s1,s2,.L7
	li	t0,4096
	add	sp,sp,t0
	lw	ra,44(sp)
	lw	s0,40(sp)
	lw	s1,36(sp)
	lw	s2,32(sp)
	lw	s3,28(sp)
	lw	s4,24(sp)
	lw	s5,20(sp)
	lw	s6,16(sp)
	lw	s7,12(sp)
	lw	s8,8(sp)
	addi	sp,sp,48
	jr	ra
	.size	rvv_fractal, .-rvv_fractal
	.ident	"GCC: () 15.1.0"
	.section	.note.GNU-stack,"",@progbits
