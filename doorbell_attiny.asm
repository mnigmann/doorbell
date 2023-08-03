#include <avr/io.h>

#define C 0x200
#define S 0x202
#define target 0x204
#define cval 0x206
#define sval 0x208
#define iter 0x20A

#define ALPHA 50545
#define ALPHA_INV 21243
#define PWM_CENTER 75

#define prodh r30
#define prodl r29
#define frac r28

#define multsl r20
#define multsh r21
#define multul r22
#define multuh r23

#define cosl r24
#define cosh r25
#define sinl r26
#define sinh r27

/*
V_t = V_{t-1} + alpha*(PWM_CENTER - PWM_{t-1})
PWM_t = PWM_CENTER + alpha_inv*(V_t - V_T) = 2*PWM_CENTER - PWM_{t-1} + alpha_inv*(V_{t-1} - V_T)
*/

.section .text

.global mult16
    .type   mult16, @function
mult16:
    ; Multiply r20:r21 (signed) by r22:r23 (unsigned) and shift by 16
    clr prodh
    clr prodl
    clr frac
    ldi r18, 15
    mult16_loop:
    lsr multsh
    ror multsl
    brcc mult16_noadd
    add prodl, multul
    adc prodh, multuh
    rjmp mult16_noclear
    mult16_noadd:
    clc
    mult16_noclear:
    ror prodh
    ror prodl
    ror frac
    dec r18
    brne mult16_loop
    
    tst multsl
    breq mult16_done
    sub prodl, multul
    sbc prodh, multuh
    ror prodh
    ror prodl
    ror frac
    ret
    mult16_done:
    lsr prodh
    ror prodl
    ror frac
    ret

.global __vector_3
    .type   __vector_3, @function
__vector_3:
    push r16
    in r16, 0x3f
    push r16
    push r17
    push r18
    push r19
    push r20
    push r21
    push r22
    push r23
    push r24
    push r25
    push r26
    push r27
    push r28
    push r29
    push r30
    push r31

    in r16, ADCL-32
    in r17, ADCH-32

    ldi multsl, lo8(PWM_CENTER)
    ldi multsh, hi8(PWM_CENTER)
    in r18, OCR0B-32
    clr r19
    mov r24, r18
    sub multsl, r18
    sbc multsh, r19                 ; r20:r21 = PWM_CENTER - OCR1B
    ldi multul, lo8(ALPHA)
    ldi multuh, hi8(ALPHA)          ; r22:r23 = ALPHA

    rcall mult16

    ldi r18, 0
    add frac, r16
    adc prodl, r17
    adc prodh, r18
    
    tst prodh
    brpl product_pos
    clr prodh
    clr prodl
    clr frac
    product_pos:
    tst prodh
    breq product_lt
    clr prodh
    ldi prodl, 0xff
    ldi frac, 0xc0
    product_lt:

    mov r26, prodl


    lsl frac
    rol prodl
    rol prodh
    lsl frac
    rol prodl
    rol prodh


    lds multsl, target
    lds multsh, target+1
    sub multsl, prodl
    sbc multsh, prodh
    ldi multul, lo8(ALPHA_INV)
    ldi multuh, hi8(ALPHA_INV)

    rcall mult16
    ldi r16, lo8(PWM_CENTER)
    ldi r17, hi8(PWM_CENTER)
    sub r16, prodl
    sbc r17, prodh
    
    brpl noclip_low
    clr r16
    clr r17
    noclip_low:
    ldi r18, 0
    cpi r16, 159
    cpc r17, r18
    brlo noclip_high
    ldi r16, 159
    noclip_high:
    ldi r18, 0b11000101
    out ADCSRA-32, r18              ; start ADC
    out OCR0B-32, r16
    
    lds r24, iter
    lds r25, iter+1
    adiw r24, 1
    sts iter, r24
    sts iter+1, r25
    ldi r26, hi8(8000)
    cpi r24, lo8(8000)
    cpc r25, r26
    brne noend
    
    ldi r24, 0b00000011             ; Disable PWM
    out TCCR0A-32, r24
    clr r25
    out DDRB-32, r25                ; Disable output
    out TIMSK-32, r25               ; Disable interrupt
    ldi r24, 0b01000000             ; Clear interrupt flags
    out TIFR-32, r24
    
    noend:
    
    /* @@@@@@@@@@@@@@@@@@@
    Sine generating algorithm
       @@@@@@@@@@@@@@@@@@@ */
    
    lds r16, cval
    lds r17, cval+1
    lds r18, sval
    lds r19, sval+1
    lds r20, C
    lds r21, C+1
    lds r22, S
    lds r23, S+1
    clr r24
    clr r25
    clr r26
    clr r27
    clr r28
    clr r29
    clr r31
    ldi r30, 15
    mul_loop:
    lsr r17
    ror r16
    brcc no_real
    add r24, r20
    adc r25, r21
    adc r26, r31
    add r27, r22
    adc r28, r23
    adc r29, r31
    no_real:
    lsr r19
    ror r18
    brcc no_imag
    sub r24, r22
    sbc r25, r23
    sbc r26, r31
    add r27, r20
    adc r28, r21
    adc r29, r31
    no_imag:
    asr r26
    ror r25
    ror r24
    asr r29
    ror r28
    ror r27
    dec r30
    brne mul_loop
        ; Do the last bit, which has a negative value. Both r16 and r18 are either 1 or 0, and r17=r19=0
    tst r16
    breq no_real_s
    sub r24, r20
    sbc r25, r21
    sbc r26, r31
    sub r27, r22
    sbc r28, r23
    sbc r29, r31
    no_real_s:
    tst r18
    breq no_imag_s
    add r24, r22
    adc r25, r23
    adc r26, r31
    sub r27, r20
    sbc r28, r21
    sbc r29, r31
    no_imag_s:
        ; Final bitshift omitted because we must multiply by 2 afterwards anyway
    sts cval, r24
    sts cval+1, r25
    sts sval, r27
    sts sval+1, r28
    
    asr r25
    ror r24
    ldi r26, 128
    add r25, r26
    clr r26
    lsl r24
    rol r25
    rol r26
    lsl r24
    rol r25
    rol r26
    sts target, r25
    sts target+1, r26
    
    isr_done:

    pop r31
    pop r30
    pop r29
    pop r28
    pop r27
    pop r26
    pop r25
    pop r24
    pop r23
    pop r22
    pop r21
    pop r20
    pop r19
    pop r18
    pop r17
    pop r16
    out 0x3f, r16
    pop r16
    reti

.global start_bell
start_bell:
    push r18
    
    sts C, cosl             ; Store sine and cosine values for this frequency
    sts C+1, cosh
    sts S, sinl
    sts S+1, sinh
    
    ldi r18, 0
    sts iter, r18           ; reset iter to 0
    sts iter+1, r18
    sts sval, r18           ; initialize sval to 0
    sts sval+1, r18
    ldi r18, 0xff           ; initialize cval to 32767
    sts cval, r18
    ldi r18, 0x7f
    sts cval+1, r18
    
    ldi r18, 0b00100011     ; Start the PWM
    out TCCR0A-32, r18
    ldi r24, 0b00000010     ; Enable the output
    out DDRB-32, r24
    ldi r18, 0b01000000     ; Enable the interrupt
    out TIMSK-32, r18
    
    bell_running:           ; Wait until the sound has finished
    in r18, TIMSK-32
    tst r18
    brne bell_running
    
    pop r18
    ret
    

.global main
    .type   main, @function
main:
    ldi r24, 0x8b
    out OSCCAL-32, r24
    
    ldi r24, 0b10000101
    out ADCSRA-32, r24      ; ADCSRA = 0b10000101
    ldi r24, 0b00001011
    out ADCSRB-32, r24      ; ADCSRB = 0b00001011
    ldi r24, 0b00100001
    out ADMUX-32, r24       ; ADMUX  = 0b00100001 (Vcc as reference, left adjust, select ADC1)
    
    ldi r24, 0b00000010
    out DDRB-32, r24

    ; TIMER1 for interrupt
    ldi r24, 0b10000100
    out TCCR1-32, r24       ; TCCR1A = 0b00100010
    ldi r25, 249
    out OCR1C-32, r25
    out OCR1A-32, r25
    clr r24
    out TIMSK-32, r24       ; Disable the interrupts

    ; TIMER0 for PWM output
    ldi r24, 159
    out OCR0A-32, r24
    ldi r24, 79
    out OCR0B-32, r24
    ldi r24, 0b00000011
    out TCCR0A-32, r24
    ldi r24, 0b00001001
    out TCCR0B-32, r24

    ldi r24, lo8(27563)
    ldi r25, hi8(27563)
    sts C, r24
    sts C+1, r25
    ldi r24, lo8(17705)
    ldi r25, hi8(17705)
    sts S, r24
    sts S+1, r25

    sei

    ldi r24, lo8(512)
    ldi r25, hi8(512)
    sts target, r24
    sts target+1, r25
    
    ldi cosl, lo8(27563)
    ldi cosh, hi8(27563)
    ldi sinl, lo8(17705)
    ldi sinh, hi8(17705)
    rcall start_bell
    ldi cosl, lo8(29419)
    ldi cosh, hi8(29419)
    ldi sinl, lo8(14412)
    ldi sinh, hi8(14412)
    rcall start_bell
    
    in r24, MCUCR-32
    andi r24, 0b11000111
    ori r24, 0b00110000
    out MCUCR-32, r24
    sleep

    mainloop:
    rjmp mainloop

