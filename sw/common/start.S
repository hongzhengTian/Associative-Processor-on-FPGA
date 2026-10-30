# Reset entry: set up gp, sp and .bss, run main(), report its return value
# as the exit code through the I/O port, then stop.

    .section .text.start, "ax"
    .global _start
_start:
    .option push
    .option norelax
    la      gp, __global_pointer$
    .option pop
    la      sp, __stack_top

    la      t0, __bss_start
    la      t1, __bss_end
1:  bgeu    t0, t1, 2f
    sw      zero, 0(t0)
    addi    t0, t0, 4
    j       1b

2:  call    main
    call    exit

    .global exit
exit:
    li      t0, 0x10000004
    sw      a0, 0(t0)
3:  j       3b
