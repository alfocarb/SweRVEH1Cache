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
        .word   64
        .word   34
        .word   25
        .word   12
        .word   22
        .word   11
        .word   90

.section .text
.globl _start

_start:
li t2, 0x008			# disable branch predictor
# csrrs t1, 0x7F9, t2

li t2, 0x028       	# blocking loads
 csrrs t1, 0x7F9, t2
# Register t3 is also called register 28 (x28)
 la t1, A			
# la t2, B					
# la t3, C	
# la t4, D			
# la t2, E					
# la t3, F
# la t1, G			
# la t2, H					
# la t3, I
# la t1, J			
# la t2, K					
# la t3, L			
# sw t2, 8(t3)
# li t2, 0xf0040000

li t2, 0x4
    nop
	nop
	nop
    la t4, A
    lw t3, 0(t4)
    add t4, t4, t2
    lw t3, 0(t4)
    add t4, t4, t2
    lw t3, 0(t4)
    add t4, t4, t2
    lw t3, 0(t4)
    add t4, t4, t2
    lw t3, 0(t4)
    add t4, t4, t2
    lw t3, 0(t4)
    add t4, t4, t2
    lw t3, 0(t4)
    add t4, t4, t2
    lw t3, 0(t4)
    add t4, t4, t2
    lw t3, 0(t4)
    add t4, t4, t2
    lw t3, 0(t4)
    add t4, t4, t2
    lw t3, 0(t4)
    add t4, t4, t2
    lw t3, 0(t4)
    add t4, t4, t2
    lw t3, 0(t4)
    add t4, t4, t2
    lw t3, 0(t4)
    add t4, t4, t2
    lw t3, 0(t4)
    add t4, t4, t2
    lw t3, 0(t4)
    add t4, t4, t2
    lw t3, 0(t4)
    add t4, t4, t2
    lw t3, 0(t4)
    add t4, t4, t2
    lw t3, 0(t4)
    add t4, t4, t2
    lw t3, 0(t4)
    add t4, t4, t2
    lw t3, 0(t4)
    add t4, t4, t2
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
 	beq  zero, zero, REPEAT1	# Repeat the loop

	nop
.end