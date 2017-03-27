/**
 * Adam Peryman (adam.peryman@gmail.com)
 * 09/04/2016
 *
 * Line drawing in ARM assembly using Bresenham's Algorithm.
 */

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
