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
@ Write Student 1’s Name here: Chen Hongyu (A0311920U)
@ Write Student 2’s Name here: Hu Xiran (A0300137X)
@ You could create a look-up table of registers here:
@ R0: N (buffer size), counter
@ R1: accel_buff (pointer to an int buffer containing the most recent samples)
@ Return (R0): integer average of the N samples
@ R2: Sum of the sample
@ R3: Values of every samples
@ R4: Total number of the samples, N (buffer size)
@ write your program from here:
mov_avg:
 PUSH {r4, lr} @ save link register and R4
 CMP r0, #0          @ Check if N less than 0
 BLE end_early       @ If N <= 0, then end early

 MOV r2, #0 @ initialise sum = 0
 MOV r4, r0 @ Copy N to R4

loop:
 LDR r3, [r1], #4 @go to the memory address indicated by r1, and load the value at this address to r3; then go to the next address (increment 4 bytes)
 ADD r2, r2, r3 @sum = sum + r3, add the value at r3 into sum
 SUBS r0, r0, #1 @counter = counter - 1, at the same time set flag if r0 = 0
 BNE loop @repeat

 SDIV r0, r2, r4 @ R0 = total_sum / N (signed division)

 end_early:
  POP {r4, pc}
