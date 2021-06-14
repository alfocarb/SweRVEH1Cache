.section .rodata
A:
    .word 0x11111111
B:
	.word 0x22222222
C:
	.word 0x33333333
D:
    .word 0x4444
E:
	.word 0x5555
F:
	.word 0x6666
G:
    .word 0x7777
H:
	.word 0x8888
I:
	.word 0x9999
J:
    .word 0xFFFFFFFF
K:
	.word 0x1111
L:
	.word 0x1212
M:
	.word 0x5555
N:
	.word 0x6666
O:
    .word 0x7777
P:
	.word 0x8888
Q:
	.word 0x9999
R:
    .word 0x1010
S:
	.word 0x1111
T:
	.word 0x1212

arr:
	.word 0x02
	.word 0x02
	.word 0x02
	.word 0x02
	.word 0x02
	.word 0x02
	.word 0x02
	.word 0x02
	.word 0x02
	.word 0x02
	.word 0x02
	.word 0x02
	.word 0x02
	.word 0x02
	.word 0x02
	.word 0x02
	.word 0x02
	.word 0x02
	.word 0x02

.section .text
.globl _start

_start:
# cache configuration
  li t1, 0x55555555
 # csrw 0x7c0, t1

li t2, 0x008			# disable branch predictor
# csrrs t1, 0x7F9, t2

li t2, 0x028       	# blocking loads
 csrrs t1, 0x7F9, t2
# Register t3 is also called register 28 (x28)

# Icache configuration
  li t1, 0x55555555
  csrw 0x7c0, t1

nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop

la a0, A
li a1, 0x4
li a2, 0x1000 //i
li a3, 0x11000
la a4, arr
# li a4, 0x02340000
FOR1:
	lw 		t1, 0(a0) //cargamos a[i]
	lw 		t3, 0(a4) //cargamos b[i]
	divw 	t2, t3, t1 //a[i]*b[i]
	sw		t2, 0(a4)
	add 	a0, a0, a1 //actualizamos punteros
	add 	a4, a4, a1 //actualizamos punteros
	add 	a3, a3, a1 //actualizamos punteros
	sub 	a2, a2, a1 //actualizamos i
 	bgt 	a2, zero, FOR1	# Repeat the loop
nop
nop
nop
nop
li t2, 0x01010101
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
 REPEAT1:
	nop
	nop
	nop
	nop
	nop
	nop
	nop
 	beq  zero, zero, REPEAT1	# Repeat the loop
	nop
	nop

.end



















































