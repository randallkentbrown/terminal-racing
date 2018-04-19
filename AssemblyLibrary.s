            TTL Start ISR Init Code
;****************************************************************
;Descriptive comment header goes here.
;(What does the program do?)
;Name:  <Your name here>
;Date:  <Date completed here>
;Class:  CMPE-250
;Section:  <Your lab section, day, and time here>
;---------------------------------------------------------------
;Keil Template for KL46 Assembly with Keil C startup
;R. W. Melton
;November 13, 2017
;****************************************************************
;Assembler directives
            THUMB
            GBLL  MIXED_ASM_C
MIXED_ASM_C SETL  {TRUE}
            OPT   64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL46Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;melton EQUates
; UART0 Interrupt EQUates
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;12:UART0 IRQ mask
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;12:UART0 IRQ pending status
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;12:UART0 IRQ mask
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port A
PORT_PCR_SET_PTA1_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTA2_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port A clock gate control (enabled)
;Use provided SIM_SCGC5_PORTA_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select
;         (PLLFLLSEL determines MCGFLLCLK' or MCGPLLCLK/2)
; 1=   16:PLLFLLSEL=PLL/FLL clock select (MCGPLLCLK/2)
SIM_SOPT2_UART0SRC_MCGPLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
SIM_SOPT2_UART0_MCGPLLCLK_DIV2 EQU \
    (SIM_SOPT2_UART0SRC_MCGPLLCLK :OR: SIM_SOPT2_PLLFLLSEL_MASK)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600  EQU  0x00
;---------------------------------------------------------------
;UART0_BDL
;0x38->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x9C
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  (UART0_S1_IDLE_MASK :OR: \
                            UART0_S1_OR_MASK :OR: \
                            UART0_S1_NF_MASK :OR: \
                            UART0_S1_FE_MASK :OR: \
                            UART0_S1_PF_MASK)
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  \
        (UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)
;---------------------------------------------------------------

; Timer Driver Equates
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ICER_PIT_MASK    EQU  PIT_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;22:PIT IRQ pending status
;12:UART0 IRQ pending status
NVIC_ICPR_PIT_MASK    EQU  PIT_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
;--PIT
PIT_IRQ_PRIORITY    EQU  0
NVIC_IPR_PIT_MASK   EQU  (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0  EQU  (PIT_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ISER_PIT_MASK    EQU  PIT_IRQ_MASK
;---------------------------------------------------------------
;PIT_LDVALn:  PIT load value register n
;31-00:TSV=timer start value (period in clock cycles - 1)
;Clock ticks for 0.01 s at 24 MHz count rate
;0.01 s * 24,000,000 Hz = 240,000
;TSV = 240,000 - 1
PIT_LDVAL_10ms  EQU  239999
;---------------------------------------------------------------
;PIT_MCR:  PIT module control register
;1-->    0:FRZ=freeze (continue'/stop in debug mode)
;0-->    1:MDIS=module disable (PIT section)
;               RTI timer not affected
;               must be enabled before any other PIT setup
PIT_MCR_EN_FRZ  EQU  PIT_MCR_FRZ_MASK
;---------------------------------------------------------------
;PIT_TCTRLn:  PIT timer control register n
;0-->   2:CHN=chain mode (enable)
;1-->   1:TIE=timer interrupt enable
;1-->   0:TEN=timer enable
PIT_TCTRL_CH_IE  EQU  (PIT_TCTRL_TEN_MASK :OR: PIT_TCTRL_TIE_MASK)
; MORE QUESRS
;PortE pin 31 symbols
PTE31_MUX_TPM0_CH4_OUT        EQU   (3 << PORT_PCR_MUX_SHIFT)
SET_PTE31_TPM0_CH4            EQU   (PORT_PCR_ISF_MASK :OR: PTE31_MUX_TPM0_CH4_OUT)            ;SIM_SOPT2 symbols
SIM_SOPT2_TPMSRC_MCGPLLCLK    EQU   (1 << SIM_SOPT2_TPMSRC_SHIFT)
SIM_SOPT2_TPM_MCGPLLCLK_DIV2  EQU   (SIM_SOPT2_TPMSRC_MCGPLLCLK :OR: \
                                     SIM_SOPT2_PLLFLLSEL_MASK)          ;TPM0_CONF symbol
                                     

;---------------------------------------------------------------
;my EQUates
IN_PTR          EQU     0
OUT_PTR         EQU     4
BUF_STRT        EQU     8
BUF_PAST        EQU     12
BUF_SIZE        EQU     16
NUM_ENQD        EQU     17
;more of my EQUates
Q_REC_SZ        EQU     18
Q_BUF_SZ        EQU     4
TxQ_BUF_SZ      EQU     80
RxQ_BUF_SZ      EQU     80
;even mroe EQUAtes
CFlagMask       EQU     0x20000000
;****************************************************************
;MACROs
;****************************************************************
;Program
;C source will contain main ()
;Only subroutines and ISRs in this assembly source
            AREA    MyCode,CODE,READONLY
            EXPORT  Init_UART0_IRQ
            EXPORT  PutStringSB
            EXPORT  GetStringSB
            EXPORT  PutNumHex
            EXPORT  PutNumU
            EXPORT  PutChar
            EXPORT  GetChar
            EXPORT  GetCharTick
            EXPORT  CheckCFlag
            EXPORT  UART0_IRQHandler
;>>>>> begin subroutine code <<<<<


; Initialize the UART0 for transmit/receive with interrupts
            ;   Inputs: None
            ;  Outputs: None
            ; Modifies: None
Init_UART0_IRQ PROC {R0-R14}
            ; Push Register values
            PUSH  {LR}
            PUSH {R0,R1,R2}
            
            LDR   R0,=TxQBuffer
            LDR   R1,=TxQRecord
            MOVS  R2,#TxQ_BUF_SZ
            BL    InitQueue                       ; Initialize the Transmit Queue
            
            LDR   R0,=RxQBuffer
            LDR   R1,=RxQRecord
            MOVS  R2,#RxQ_BUF_SZ
            BL    InitQueue                       ; Initialize the Receive Queue
            
            
            ;Select MCGPLLCLK / 2 as UART0 clock source     
            LDR   R0,=SIM_SOPT2
            LDR   R1,=SIM_SOPT2_UART0SRC_MASK
            LDR   R2,[R0,#0]
            BICS  R2,R2,R1
            LDR   R1,=SIM_SOPT2_UART0_MCGPLLCLK_DIV2
            ORRS  R2,R2,R1
            STR   R2,[R0,#0]
            ;Enable external connection for UART0
            LDR   R0,=SIM_SOPT5
            LDR   R1,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
            LDR   R2,[R0,#0]
            BICS  R2,R2,R1
            STR   R2,[R0,#0]
            ;Enable clock for UART0 module
            LDR   R0,=SIM_SCGC4
            LDR   R1,=SIM_SCGC4_UART0_MASK
            LDR   R2,[R0,#0]
            ORRS  R2,R2,R1
            STR   R2,[R0,#0]
            ;Enable clock for Port A module
            LDR   R0,=SIM_SCGC5 
            LDR   R1,=SIM_SCGC5_PORTA_MASK  
            LDR   R2,[R0,#0]   
            ORRS  R2,R2,R1   
            STR   R2,[R0,#0] 
            ;Connect PORT A Pin 1 (PTA1) to UART0 Rx (J1 Pin 02)  
            LDR   R0,=PORTA_PCR1   
            LDR   R1,=PORT_PCR_SET_PTA1_UART0_RX   
            STR   R1,[R0,#0]
            ;Connect PORT A Pin 2 (PTA2) to UART0 Tx (J1 Pin 04)   
            LDR   R0,=PORTA_PCR2   
            LDR   R1,=PORT_PCR_SET_PTA2_UART0_TX   
            STR   R1,[R0,#0] 
 
            ; Load UART0_Base
            LDR     R0,=UART0_BASE
            ; Disable UART0
            MOVS    R1,#UART0_C2_T_R
            LDRB    R2,[R0,#UART0_C2_OFFSET]
            BICS    R2,R2,R1
            STRB    R2,[R0,#UART0_C2_OFFSET]
            
            ;Set UART0 IRQ priority    
            LDR     R0,=UART0_IPR    
            ;LDR     R1,=NVIC_IPR_UART0_MASK    
            LDR     R2,=NVIC_IPR_UART0_PRI_3    
            LDR     R3,[R0,#0]    
            ;BICS    R3,R3,R1    
            ORRS    R3,R3,R2    
            STR     R3,[R0,#0] 
            ;Clear any pending UART0 interrupts    
            LDR     R0,=NVIC_ICPR    
            LDR     R1,=NVIC_ICPR_UART0_MASK    
            STR     R1,[R0,#0] 
            ;Unmask UART0 interrupts    
            LDR     R0,=NVIC_ISER    
            LDR     R1,=NVIC_ISER_UART0_MASK    
            STR     R1,[R0,#0] 
            
            ; Set UART0 Baud Rate
            LDR   R0,=UART0_BASE
            MOVS  R1,#UART0_BDH_9600
            STRB  R1,[R0,#UART0_BDH_OFFSET]
            MOVS  R1,#UART0_BDL_9600
            STRB  R1,[R0,#UART0_BDL_OFFSET]
            ; Set the bit transmission format
            MOVS  R1,#UART0_C1_8N1
            STRB  R1,[R0,#UART0_C1_OFFSET]
            MOVS  R1,#UART0_C3_NO_TXINV
            STRB  R1,[R0,#UART0_C3_OFFSET]
            MOVS  R1,#UART0_C4_NO_MATCH_OSR_16
            STRB  R1,[R0,#UART0_C4_OFFSET]
            MOVS  R1,#UART0_C5_NO_DMA_SSR_SYNC
            STRB  R1,[R0,#UART0_C5_OFFSET]
            MOVS  R1,#UART0_S1_CLEAR_FLAGS
            STRB  R1,[R0,#UART0_S1_OFFSET]
            MOVS  R1,#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
            STRB  R1,[R0,#UART0_S2_OFFSET]
            
            ; Enable UART0
            MOVS  R1,#UART0_C2_T_RI
            STRB  R1,[R0,#UART0_C2_OFFSET]
            ; Pop and return
            POP {R0,R1,R2}
            POP {PC}
            ENDP

; PutStringSB subroutine, displays a null terminated string (up to R1 characters) starting from the address in R0
            ; Inputs: R0, R1
            ; R0: Starting Memory Address
            ; R1: Buffer Size
            ; Output: None
PutStringSB PROC    {R0-R14}    
            PUSH    {LR}        ; Push the Link Register
            PUSH    {R0,R1,R2}    ; Push the low registers
            MOVS    R2,R0       ; Move the address from R0 into R2, so the address can be incremented, and R0 can be used for characters
            
PSLoop      LDRB    R0,[R2,#0]  ; Load the byte at memory address R2
            BL      PutChar     ; Transmit the loaded character to the terminal
            SUBS    R1,R1,#1    ; Subtract 1 from the remaining available buffer capacity
            ADDS    R2,R2,#1    ; Increment the memory address in R2 to print the next byte
            CMP     R1,#0       ; Compare R1 to 0
            BEQ     PSOverrun   ; If no buffer space remains, branch to the overrun state
            CMP     R0,#0        ; Compare R0 to the NUL character
            BEQ     PSDone      ; The loop is finished if the most recently printed character was a NUL
            B       PSLoop      ; Return to the start of the loop
            
PSOverrun   MOVS    R0,#0        ; Move the NUL character into R0
            BL      PutChar     ; Print the NUL character
            
PSDone      NOP                 ; No operation is required after the loop of the subroutine is done

PSComplete  POP     {R0,R1,R2}  ; Pop the low registers
            POP     {PC}        ; Return the the linked address
            ENDP

; GetStringSB subroutine, gets a string for R1 characters and places it at R0
            ; Input
            ; R0: Starting Memory Address
            ; R1: Buffer Size
            ; Output
            ; None
GetStringSB PROC    {R0-R14}
            PUSH    {LR}        ; Push the Link Register
            PUSH    {R0,R1,R2}    ; Push used low registers
            MOVS    R2,R0       ; Put R0 into R2 (to save memory address after GetChar is called)

GetStringSB_Loop      
            BL      GetChar     ; GetChar
            CMP     R0,#13      ; Determine if the received character was a Carriage Return
            BEQ     GetStringSB_Done
            CMP     R0,#'\b'    ; Determine if the received character was a backspace
            BEQ     GetStringSB_Backspace
            CMP     R0,#0x7F    ; Determine if the received character was a delete
            BEQ     GetStringSB_Backspace
            CMP     R1,#0        ; Compare R1 to 0
            BLE     GetStringSB_Loop
            SUBS    R1,R1,#1    ; Decrease the remaining buffer size by 1
            BL      PutChar     ; Echo the character
            STRB    R0,[R2,#0]  ; Store the read character in the memory located at R2
            ADDS    R2,R2,#1    ; Increment Memory Location of R2 by one (to store at next byte).
            B       GetStringSB_Loop
GetStringSB_Backspace
            ADDS    R1,R1,#1
            SUBS    R2,R2,#1
            BL      PutChar
            B       GetStringSB_Loop
GetStringSB_Done
            MOVS    R0,#0        ; Move 0 (aka NUL) into R0
            STRB    R0,[R2,#0]  ; Store NUL at address R2
            MOVS    R0,#10        ; Move a Newline into R0
            BL      PutChar     ; Put a Newline into the terminal
            MOVS    R0,#13      ; Move a Carriage Return into R0
            BL      PutChar     ; Put a Carriage Return into the terminal
            
GetStringSB_Complete  
            POP     {R0,R1,R2}  ; Pop low registers
            POP     {PC}        ; Pop PC to return to previous spot
            ENDP                ; End the PROC statement

; PutChar subroutine, poll for TDRE ready, then transmit characters.
PutChar     PROC  {R0-R12}
            ; Push onto stack
            PUSH  {LR}
            PUSH  {R1,R2,R3}

            ; Subroutine Code
PutChar_AttemptEnqueue
            CPSID I             ; Disable interrupts
            LDR   R1,=TxQRecord ; Load queue record
            BL    Enqueue       ; Enqueue character
            CPSIE I             ; Enable interrupts
            NOP                 ; Do I need this??
            BCS   PutChar_AttemptEnqueue

            ; Set the interrupt flag
            LDR   R1,=UART0_BASE
            MOVS  R2,#UART0_C2_TI_RI
            STRB  R2,[R1,#UART0_C2_OFFSET]
            
            ; Pop and leave
            POP {R1,R2,R3}
            POP {PC}
            ENDP

; GetChar subroutine, poll for a character from the serial port
GetChar     PROC {R1-R12}
            ; Push and start
            PUSH {LR}
            PUSH {R1}

            ; Subroutine code
GetChar_TryDequeue
            CPSID I             ; Disable interrupts
            LDR   R1,=RxQRecord ; Load queue record
            BL    Dequeue       ; Try to dequeue a character
            CPSIE I             ; Enable interrupts
            BCS   GetChar_TryDequeue

            ; Pop and leave
            POP {R1}
            POP {PC}
            ENDP
                
; GetCharTick Subroutine
            ; Subroutine to get a character (attempts only once, then quits out)
            ; Inputs:  None
            ; Outputs: R0
            ; R0: The character read from the UART0
            ; Modifies: APSR
            ; APSR: C flag set if no character was dequeued.
GetCharTick PROC {R1-R12}
            ; Push and start
            PUSH {LR}
            PUSH {R1}

            ; Subroutine code
GetCharTick_TryDequeue
            CPSID I             ; Disable interrupts
            LDR   R1,=RxQRecord ; Load queue record
            BL    Dequeue       ; Try to dequeue a character
            CPSIE I             ; Enable interrupts
            ;BCS   GetChar_TryDequeue

            ; Pop and leave
            POP {R1}
            POP {PC}
            ENDP

; UART0_ISR reacts to a UART0 Interrupt.
UART0_IRQHandler   PROC  {R0-R12}
            CPSID I   ; Mask interrupts.
            PUSH    {LR}
            LDR   R2,=UART0_BASE
            LDRB  R0,[R2,#UART0_C2_OFFSET]
            LDRB  R3,[R2,#UART0_S1_OFFSET]
            MOVS  R1,#UART0_C2_TIE_MASK
            TST   R0,R1                       ; Determine if the Transmit interrupt is set.
            BEQ   UART0_ISR_Receive          ; Branch to Transmit if the Transmit interrupt is set.
            MOVS  R1,#UART0_S1_TDRE_MASK
            TST   R3,R1                       ; Determine if TDRE is set
            BEQ   UART0_ISR_Receive
            LDR   R1,=TxQRecord
            BL    Dequeue                     ; Attempt to dequeue a character from the TxQ
            BCS   UART0_ISR_Transmit_Disable
            STRB  R0,[R2,#UART0_D_OFFSET]      ; Write the character to the transmit data register
            B     UART0_ISR_Receive
UART0_ISR_Transmit_Disable
            MOVS  R1,#UART0_C2_T_RI
            STRB  R1,[R2,#UART0_C2_OFFSET]
UART0_ISR_Receive
            MOVS  R1,#UART0_S1_RDRF_MASK
            TST   R3,R1                       ; Determine if RDRF is set
            BEQ   UART0_ISR_Done              ; Branch to Done if RDRF is not set
            LDRB  R0,[R2,#UART0_D_OFFSET]
            LDR   R1,=RxQRecord
            BL    Enqueue
UART0_ISR_Done
            CPSIE I   ; Enable interrupts.
            POP     {PC}
            ENDP      ; End the interrupt service.

; Initialize the Queue
            ; Inputs
            ; R0: Queue Buffer address
            ; R1: Queue Record address
            ; R2: Queue character capacity
            ; Outputs
            ; NONE
InitQueue   PROC     {R0-R14}         ; Process does not change R0-R14
            PUSH     {LR}                    ; Save the link register
            PUSH    {R0-R2}                  ; Save the low registers
            
InitQueue_Init
      STR        R0,[R1,#IN_PTR]          ; Store the address of the first spot in the queue at the InPointer
            STR        R0,[R1,#OUT_PTR]      ; Store the address of the first spot in the queue at the OutPointer
            STR     R0,[R1,#BUF_STRT]      ; Store the address of the first spot in the queue at the BufferStart
            ADDS    R0,R0,R2                  ; Increment the address in R0 to represent the first address outside the queue
            STR        R0,[R1,#BUF_PAST]     ; Store the address of the first spot outside the queue into BufferPast
            STRB    R2,[R1,#BUF_SIZE]     ; Store the size of the queue in the BufferSize
            MOVS    R2,#0                        ; Move 0 into R2
            STRB    R2,[R1,#NUM_ENQD]     ; Set the current size of the queue equal to 0.
            
InitQueue_Done
            POP     {R0-R2}                  ; Restore low registers
            POP     {PC}                    ; Return to linked address
      ENDP                              ; End process


; Dequeue an item from the queue.
            ; Inputs
            ; R1: Queue Record Structure Address
            ; Outputs
            ; R0: Dequeued Character (byte)
Dequeue        PROC     {R1-R14}             ; Process does not change R0-R14
            PUSH     {LR}                        ; Save the link register
            PUSH    {R1-R3}                      ; Save the low registers
Dequeue_CE                                      ; = Check to see if the queue is empty
            LDRB    R2,[R1,#NUM_ENQD]      ; Get the address of the number of enqueued characters
            CMP     R2,#0                        ; Determine if no characters are in the queue
            BLE        Dequeue_EMPTY            ; If the queue is empty, move to the DQ_EMPTY label
Dequeue_CONTINUE                            ; = Continue with the subroutine
            LDR     R2,[R1,#OUT_PTR]      ; Get the address of the output slot of the queue
            LDRB    R0,[R2,#0]                ; Get the character from the output of the queue

            LDR        R2,[R1,#OUT_PTR]      ; Load the OutPointer
            ADDS     R2,R2,#1                  ; Increment the address of the OutPointer by one byte
            LDR        R3,[R1,#BUF_PAST]      ; Load the address of the first spot past the end of the buffer
            CMP        R2,R3                        ; Compare the incremented address to the first address past
            BLO        Dequeue_STORE_INC      ; Branch to the address storage label
            LDR        R2,[R1,#BUF_STRT]      ; Change the value of the InPointer to be the first address in the queue (iff the inpointer overran)
Dequeue_STORE_INC                            ; = Store the incremented address
            STR        R2,[R1,#OUT_PTR]      ; Store the incremented address of the OutPointer
            
            LDRB     R2,[R1,#NUM_ENQD]      ; Load the number of enqueued values
            SUBS    R2,R2,#1                  ; Decrement the number of enqueued values
            STRB    R2,[R1,#NUM_ENQD]      ; Store the decremented number of enqueued values
Dequeue_SUCCESS                                ; = Clear the C flag if the dequeue is successful
            MOVS     R2,#0                        ; Place a 0 in R2
            LSRS    R2,R2,#1                  ; Move that 0 into the C flag
            B        Dequeue_DONE              ; Go to the end of the subroutine
Dequeue_EMPTY                                  ; = Set the C flag if the dequeue is not successful
            MOVS    R2,#1                        ; Put a 1 in R2
            LSRS    R2,R2,#1                  ; Move that 1 into the C flag
Dequeue_DONE                                  ; = End the subroutine
            POP     {R1-R3}                      ; Restore low registers
            POP     {PC}                        ; Return to linked address
            ENDP                            ; End process

            
; Enqueue an item into the queue
            ; Inputs
            ; R0: the character to enqueue
            ; R1: the queue record structure address
            ; Outputs
            ; C flag is changed
Enqueue     PROC     {R0-R14}             ; Process does not change R0-R14
            PUSH     {LR}                ; Save the link register
            PUSH    {R0-R3}                ; Save the low registers
Enqueue_CF                                ; = Check to see if the queue is full
            LDRB    R2,[R1,#NUM_ENQD]    ; Get the address of the number of enqueued characters
            CMP     R2,#Q_BUF_SZ        ; Determine if the queue is full
            BHS        Enqueue_FULL        ; If the queue is full, move to the Enqueue_FULL label
Enqueue_CONTINUE                        ; = Continue with the subroutine
            LDR     R2,[R1,#IN_PTR]        ; Get the address of the input slot of the queue
            STRB    R0,[R2,#0]            ; Store the character from the input into the queue
            
            LDR        R2,[R1,#IN_PTR]        ; Load the InPointer
            ADDS     R2,R2,#1            ; Increment the address of the InPointer by one byte
            LDR        R3,[R1,#BUF_PAST]    ; Load the address of the first spot past the end of the buffer
            CMP        R2,R3                ; Compare the incremented address the first address past
            BLO        Enqueue_STORE_INC    ; Branch to the address storage label
            LDR        R2,[R1,#BUF_STRT]    ; Change the value of the InPointer to be the first address in the queue (iff the inpointer overran)
Enqueue_STORE_INC                        ; = Store the incremented address
            STR        R2,[R1,#IN_PTR]        ; Store the incremented address of the OutPointer
            
            LDRB     R2,[R1,#NUM_ENQD]    ; Load the number of enqueued values
            ADDS    R2,R2,#1            ; Decrement the number of enqueued values
            STRB    R2,[R1,#NUM_ENQD]    ; Store the decremented number of enqueued values
Enqueue_SUCCESS                            ; = Clear the C flag if the enqueue is successful
            MOVS     R2,#0                ; Place a 0 in R2
            LSRS    R2,R2,#1            ; Move that 0 into the C flag
            B        Enqueue_DONE        ; Go to the end of the subroutine
Enqueue_FULL                            ; = Set the C flag if the enqueue is not successful
            MOVS    R2,#1                ; Put a 1 in R2
            LSRS    R2,R2,#1            ; Move that 1 into the C flag
Enqueue_DONE
            POP     {R0-R3}                ; Restore low registers
            POP     {PC}                ; Return to linked address
            ENDP


; Display the hex value in R0
PutNumHex   PROC  {R0-R14}    ; Process does not change R0-R14
            PUSH  {LR}        ; Save the link register
            PUSH  {R0-R2}          ; Save the low registers

            MOVS    R1,#0xF   ; Create a mask
            MOVS    R2,#32
            MOVS    R4,R0     ; Save R0
PutNumHex_Loop
            SUBS    R2,R2,#4  ; Decrement
            MOVS    R0,R4
            LSRS    R0,R0,R2
            ANDS    R0,R0,R1
            ADDS    R0,R0,#'0'
            CMP     R0,#'9'
            BLS     PutNumHex_NonALPH
            ADDS    R0,R0,#7
PutNumHex_NonALPH
            BL      PutChar
            CMP     R2,#0
            BNE     PutNumHex_Loop

            POP     {R0-R2}   ; Restore low registers
            POP     {PC}      ; Return to linked address
            ENDP              ; End process
              
              
; PutNumU subroutine, displays an unsigned integer
            ; Input
            ; R0: the unsigned int to display
PutNumU     PROC    {R0-R14}    ; PROC
            PUSH    {LR}        ; Push the Link Register
            PUSH    {R0,R1,R2}  ; Push the low registers
            MOVS    R2,#0       ; No powers of 10 recorded
            MOVS    R1,R0       ; Put R0 into R1 (for valid DIVU input parameters)
            
PNULoop     MOVS    R0,#10      ; Move 10 into R1. It will be the only thing you divide by.
            BL      DIVU        ; Divide R0 by 10.
            PUSH    {R1}        ; Push R1 onto the stack
            ADDS    R2,R2,#1    ; Another division occured
            CMP     R0, #0      ; Compare R0 to 0
            BEQ     PNUDone     ; If the quotient is 0, the last digit has been divided out
            MOVS    R1,R0       ; Get the quotient from the last calculation, and make it the divisor
            B       PNULoop     ; Return, and complete another iteration
            
PNUDone     POP     {R1}        ; Pop R1 from the stack
            MOVS    R0,R1       ; Move R1 into R0
            ADDS    R0,R0,#0x30; Convert the remainder value into a character
            BL      PutChar     ; Print out the character form of the number
            SUBS    R2,R2,#1    ; Remove 1 from the character counter
            CMP     R2,#0       ; Determine the number of remaining characters
            BGT     PNUDone     ; If characters remain, return to PNUDone

PNUComplete POP     {R0,R1,R2}  ; Pop the low registers
            POP     {PC}        ; Return to the linked address

            ENDP


; DIVU subroutine. Divides R1 by R0.
            ; Input
            ; R0: what R1 will be divided by.
            ; R1: R1 what gets divided
            ; Output
            ; R0: quotient
            ; R1: remainder
DIVU        PROC    {R2-R14}
            PUSH    {LR}
            PUSH    {R2,R3}     ; Push R2 onto the stack, it will become the divisor.
            CMP     R0,#0       ; Ensure the divisor is not 0
            BEQ     DIVU_SP_0   ; Move R0 into R2
            MOVS    R2,R0       ; Make R0 0
            MOVS    R0,#0
            
DIVU_1      CMP     R1,R2       ; If divisor is greater than dividend, you're done.
            BLO     DIVU_DONE   ; Otherwise, subtract divisor from dividend.
            SUBS    R1,R1,R2
            ADDS    R0,R0,#1    ; Return to the top of the loop.
            B       DIVU_1      
            
DIVU_DONE   NOP                 ; You're done!
            ; Clear the CLEAR flag.
            MRS     R2,APSR
            MOVS    R3,#0x20
            LSLS    R3,R3,#24
            BICS    R2,R2,R3
            MSR     APSR,R2
            ; Return R2's original value to the register.
            POP     {R2,R3}
            ; Return to the branched address.
            POP     {PC}
            
DIVU_SP_0   NOP                 ; Set the CLEAR flag!
            MRS     R2,APSR
            MOVS    R3,#0x20
            LSLS    R3,R3,#24
            ORRS    R2,R2,R3
            MSR     APSR,R2
            POP     {R2,R3}     ; Pop R2 and R3
            POP     {PC}
                                ; End case divisor = 0.
            ENDP                ; End the DIVU subroutine.
                
; Subroutine to check the C flag
            ; Inputs: None
            ; Outputs: R0
            ; R0: 1 if C flag is Set, 0 if C flag is clear
            ; Modify: None
CheckCFlag  PROC    {R1-R14}
            PUSH    {LR}
            PUSH    {R1,R2}

            MRS     R1,APSR
            LDR     R2,=CFlagMask
            ANDS    R1,R2,R1
            TST     R1,R2
            BEQ     CheckCFlag_CSet
CheckCFlag_CClear
            MOVS     R0,#0
            B       CheckCFlag_Done
CheckCFlag_CSet
            MOVS     R0,#1
CheckCFlag_Done
            POP     {R1,R2}
            POP     {PC}
            ENDP    


;>>>>>   end subroutine code <<<<<
            ALIGN
;**********************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
;>>>>>   end constants here <<<<<
;**********************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
TxQRecord     SPACE   Q_REC_SZ
              ALIGN
TxQBuffer     SPACE   TxQ_BUF_SZ
              ALIGN
RxQRecord     SPACE   Q_REC_SZ
              ALIGN
RxQBuffer     SPACE   RxQ_BUF_SZ
              ALIGN
;>>>>>   end variables here <<<<<
            END