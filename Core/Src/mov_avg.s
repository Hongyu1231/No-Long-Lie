/*
 * mov_avg.s
 *
 * Created on: 2/2/2026
 * Author: Hitesh B, Hou Linxin
 */
.syntax unified
 .cpu cortex-m4
 .thumb
 .global mov_avg
 .equ N_MAX, 8
 .bss
 .align 4

 .text
 .align 2
@ CG2028 Assignment, Sem 2, AY 2025/26
@ (c) ECE NUS, 2025
@ Write Student 1’s Name here: ABCD (A1234567R)
@ Write Student 2’s Name here: WXYZ (A0000007X)
@ You could create a look-up table of registers here:
@ R0 ...
@ R1 ...
@ write your program from here:
mov_avg:
 PUSH {r2-r11, lr}

 POP {r2-r11, pc}
