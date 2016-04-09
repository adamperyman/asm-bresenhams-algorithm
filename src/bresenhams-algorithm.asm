/**
 * Adam Peryman (adam.peryman@gmail.com)
 * 09/04/2016
 *
 * Line drawing in ARM assembly using Bresenham's Algorithm.
 */

            .global _start
            .equ cx,320                 @ Centre of the screen.
            .equ cy,240                 @ (320 x 240).
            .equ sz,(120<<16)           @ Size of cube.

_start:     bl hardware_init            @ Initialise the hardware.

            ldr r8,=0x07800000          @ Frame buffer 0.
            ldr r9,=0x07900000          @ Frame buffer 1.
            adr r10,cube0               @ Pointer to cube data.
            adr r11,cube1               @ Cube data for fb1.
            ldr r12,=0x7fff             @ Set colour.

redo:       mov r0,r8                   @ Set fb0.
            mov r1,r10                  @ Set cube0.
            mov r2,r12                  @ Initialise colour.
            bl cube                     @ Draw cube0 in frame buffer.

            bl fb_setbuffer             @ Flip the buffer.

            mov r0,r9                   @ Set fb1.
            mov r1,r11                  @ Set cube1.
            mov r2,#0                   @ Change colour to black.
            bl cube                     @ Undraw cube in fb1.

            mov r0,r10                  @ Update cube0.
            mov r1,r11                  @ Put it in cube1.
            bl movecube

            mov r0,r9                   @ Set fb1.
            mov r1,r11                  @ Set cube1.
            mov r2,r12                  @ Set colour.
            bl cube                     @ Draw cube1 in fb1.

            bl fb_setbuffer             @ Flip the buffer to fb1.

            mov r0,r8                   @ Set fb0.
            mov r1,r10                  @ Set cube0.
            mov r2,#0                   @ Change colour to black.
            bl cube                     @ Undraw cube in fb0.

            mov r0,r11                  @ Update cube1.
            mov r1,r10                  @ Put it in cube0.
            bl movecube

            b redo                      @ Loop back.

movecube:   stmfd sp!,{r0-r8,lr}        @ Move the cube.
            mov r2,r0
            mov r5,#8                   @ 8 points.

moveloop:   ldr r3,[r2]                 @ Load x, y, and z coords.
            ldr r4,[r2,#4]
            ldr r8,[r2,#8]
            add r6,r3,r4,asr#12         @ x = x + y / 4096
            sub r7,r4,r3,asr#12         @ y = y - x / 4096
            add r7,r7,r8,asr#13         @ y = y + x / 8192
            sub r8,r8,r4,asr#13         @ z = z - y / 8192
            add r6,r6,r8,asr#14         @ x = x + z / 16384
            sub r8,r8,r3,asr#14         @ z = z - x / 16384
            str r6,[r1]                 @ Store x, y, and z coords.
            str r7,[r1,#4]
            str r8,[r1,#8]
            add r2,r2,#12               @ Move to next point.
            add r1,r1,#12
            subs r5,r5,#1
            bne moveloop
            ldmfd sp!,{r0-r8,pc}

@ Draw a cube in fb given by r0 with 8 points pointed to by r1 and
@ colour in r2.

cube:       stmfd sp!,{r0-r12,lr}
            mov r8,r1                   @ Save pointer.
            mov r12,r2                  @ Save colour.
            mov r9,#0                   @ 8 points.

cloop1:     add r10,r9,#1               @ Look at other points.

cloop2:     eors r5,r9,r10              @ Find bit difference.
            beq noline                  @ Dont draw if 0.
            sub r1,r5,#1                @ Clear single bit.
            ands r5,r5,r1               @ Using v = v & (v-1).
            bne noline                  @ Dont draw if more than one bit is set.

            add r11,r8,r9,lsl#3         @ Get address of start coords
            add r11,r11,r9,lsl#2        @ by multiplying index * 12.
            ldrsh r1,[r11,#2]           @ Get top 16 bits of x coord.
            ldrsh r2,[r11,#6]           @ Get top 16 bits of y coord.
            ldrsh r3,[r11,#10]          @ Get top 16 bits of z coord.
            mul r6,r3,r1                @ Adjust perspective.
            add r1,r6,asr#9             @ x = x + (z * x / 512)
            mul r6,r3,r2
            add r2,r6,asr#9             @ y = y + (z * y / 512)

            add r11,r8,r10,lsl#3        @ Get address of end coords
            add r11,r11,r10,lsl#2       @ by multiplying index * 12.
            ldrsh r3,[r11,#2]           @ Get top 16 bits of the x coord.
            ldrsh r4,[r11,#6]           @ Get top 16 bits of the y coord.
            ldrsh r5,[r11,#10]          @ Get top 16 bits of the z coord.
            mul r6,r5,r3                @ Adjust perspective.
            add r3,r6,asr#9             @ x = x + (z * x / 512)
            mul r6,r5,r4
            add r4,r6,asr#9             @ y = y + (z * y / 512)

            mov r5,r12                  @ Set colour.
            add r1,r1,#cx               @ Move to centre.
            add r2,r2,#cy
            add r3,r3,#cx
            add r4,r4,#cy
            bl .Line                    @ Draw line.

noline:     add r10,r10,#1              @ Increment end point index.
            cmp r10,#8                  @ while != 8
            bne cloop2
            add r9,r9,#1                @ Increment start point index.
            cmp r9,#7                   @ while != 7
            bne cloop1
            ldmfd sp!,{r0-r12,pc}

cube0:      .word -sz,-sz,-sz           @ Cube starting position.
            .word -sz,-sz,sz
            .word -sz,+sz,-sz
            .word -sz,+sz,sz
            .word +sz,-sz,-sz
            .word +sz,-sz,sz
            .word +sz,sz,-sz
            .word +sz,sz,sz

cube1:      .space 8*12                 @ Space for other cube.

/************************************************************
 *                     Fun starts here!                     *
 ************************************************************/

@ For readability.
            fb .req r0
            x0 .req r1
            y0 .req r2
            x1 .req r3
            y1 .req r4
            colour .req r5
            steep .req r6
            deltax .req r7
            deltay .req r8
            error .req r9
            ystep .req r10
            x .req r11
            y .req r12

@ void line(char* fb, int x0, int y0, int x1, int y1, int colour)
.Line:
            stmfd sp!,{r0-r12,lr}       @ Push working regs to stack.

                                        @ steep = abs(y1-y0) > abs(x1-x0)
            subs y,y1,y0                @ y = y1-y0
            rsbmi y,y,#0                @ abs(y1-y0) if necessary.

            subs x,x1,x0                @ x = x1-x0
            rsbmi x,x,#0                @ abs(x1-x0) if necessary.

            cmp y,x                     @ if (y > x)
            movgt steep,#1              @   steep = true
                                        @ else
            movle steep,#0              @   steep = false

            cmp steep,#1                @ if (steep)
            eoreq x0,x0,y0              @   std::swap(x0, y0)
            eoreq y0,x0,y0
            eoreq x0,x0,y0
            eoreq x1,x1,y1              @   std::swap(x1, y1)
            eoreq y1,x1,y1
            eoreq x1,x1,y1

            cmp x0,x1                   @ if (x0 > x1)
            eorgt x0,x0,x1              @   std::swap(x0, x1)
            eorgt x1,x0,x1
            eorgt x0,x0,x1
            eorgt y0,y0,y1              @   std::swap(y0, y1)
            eorgt y1,y0,y1
            eorgt y0,y0,y1

            sub deltax,x1,x0            @ deltax = x1 - x0

            subs deltay,y1,y0           @ deltay = y1-y0
            rsbmi deltay,deltay,#0      @ abs(y1-y0) if necessary.

            mov error,deltax,lsr#1      @ error = deltax / 2

            mov y,y0                    @ y = y0

            cmp y0,y1                   @ if (y0 < y1)
            movlt ystep,#1              @   ystep = 1
                                        @ else
            movge ystep,#-1             @   ystep = -1

            mov x,x0                    @ Initialize x to x0.
            bl .drawLine                @ Go to draw loop.

                                        @ Finish.
            ldmfd sp!,{r0-r12,pc}       @ Pop original registers.

@ for (x = x0; x <= x1; ++x)
.drawLine:
            stmfd sp!,{lr}              @ Push link register.

            cmp steep,#1                @ if (steep)
            eoreq x,x,y                 @   std::swap(x, y)
            eoreq y,x,y
            eoreq x,x,y

            bl .drawPixel               @ ..Draw a pixel.

            cmp steep,#1                @ Swap back if necessary.
            eoreq x,x,y
            eoreq y,x,y
            eoreq x,x,y

            subs error,error,deltay     @ error -= deltay

                                        @ if (error < 0)
            addlt y,y,ystep             @   y += ystep
            addlt error,error,deltax    @   error += deltax


            ldmfd sp!,{lr}              @ Pop link register.
            cmp x,x1                    @ if (x > x1)
            movgt pc,lr                 @   return
                                        @ else
            add x,x,#1                  @   increment x
            b .drawLine                 @   loop again

@ void Plot(char* fb, int x, int y, int colour)
.drawPixel:
            stmfd sp!,{r6-r7}           @ Push working regs to stack.

                                        @ Calculate address.
            mov r6,#1280                @ r6 = 1280
            mul r7,y,r6                 @ r7 = y * 1280
            add r7,fb,r7                @ r7 = fb + (y * 1280)
            add r6,x,lsl#1              @ r6 = x * 2
            add r7,r7,r6                @ r7 = fb + (y * 1280) + (x * 2)

            strh colour,[r7]            @ Store the address of the pixel.

            ldmfd sp!,{r6-r7}           @ Pop original registers.
            mov pc,lr                   @ Return.

@ Clean up.
            .unreq fb
            .unreq x0
            .unreq y0
            .unreq x1
            .unreq y1
            .unreq colour
            .unreq steep
            .unreq deltax
            .unreq deltay
            .unreq error
            .unreq ystep
            .unreq x
            .unreq y
