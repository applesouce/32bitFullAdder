			TTL Exercise 08 
;****************************************************************
;Descriptive comment header goes here.
;(What does the program do?)
;Name:  Andrew Saridakis
;Date:  
;Class:  CMPE-250
;Section: L3, Wednseday, 5:00-7:00
;---------------------------------------------------------------
;Keil Template for KL05
;R. W. Melton
;September 13, 2020
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL05Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;****************************************************************
;EQUates

;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port B
PORT_PCR_SET_PTB2_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTB1_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
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
;01=27-26:UART0SRC=UART0 clock source select (MCGFLLCLK)
;---------------------------------------------------------------
SIM_SOPT2_UART0SRC_MCGFLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
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
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;26->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
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
UART0_C2_T_R  EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
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
; Queue management record field offsets

NUMBER_OF_BITS   EQU   128    ; Used for Num64
MAX_STRING  EQU   79     ; Used for past Methods 
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
            IMPORT  Startup
Reset_Handler  PROC  {}
main
;---------------------------------------------------------------
;Mask interrupts
            CPSID   I
;KL05 system startup with 48-MHz system clock
            BL      Startup
;---------------------------------------------------------------
;>>>>> begin main program code <<<<<
            BL Init_UART0_Polling
			;BASIC TESTING                 COMPLETE!!!!!!!!!!!!!
			; This section of main should test the get HEx Multi and
			; the Put hex int multi
;			LDR R0,=Prompt1
;			MOVS R1,#MAX_STRING
;			BL PutStringSB
;			
;		    MOVS R5, #NUMBER_OF_BITS; NEED the Counter 32 Hex digits - 128 /4 or NUMBER_OF /
;            LSRS R1, R5, #6 ; R1 is the input for Get Hex Int Multi, it indicates that we are using 2 words
;			LDR  R0,=FirstHex
;            BL GetHexIntMulti
;			
;			LDR R0,=SumPrompt
;			MOVS R1,#MAX_STRING
;			BL PutStringSB
;			LDR  R0,=FirstHex    ; Loading the value added
;		    LSRS R1, R5, #6      ; Used to Get the Number of Words
;            BL PutHexIntMulti
           ;BASIC TESTING                  COMPLETE!!!!!!!!!!!!

            B skipReintialized	
Again1			
            MOVS R0, #0x0D    ; (CR) moves cursor to new line
			BL PutChar
			MOVS R0, #0x0A    ; (LF)line feed to terminal screen
			BL PutChar
;*****************Prompts the User****************
skipReintialized
            LDR R0,=Prompt1
			MOVS R1,#MAX_STRING
			BL PutStringSB
;*****************Prompts the User****************
			
			MOVS R5, #NUMBER_OF_BITS; NEED the Counter 32 Hex digits - 128 /4 or NUMBER_OF /4
InvalidFirst
            LSRS R1, R5, #5 ; R1 is the input for Get Hex Int Multi, it indicates that we are using 4 words
			LDR  R0,=FirstHex
            BL GetHexIntMulti
			
			BCC NextPrompt
			MOVS R0, #0x0D    ; (CR) moves cursor to new line
			BL PutChar
			MOVS R0, #0x0A    ; (LF)line feed to terminal screen
			BL PutChar
			LDR R0,=Faliure
			MOVS R1,#MAX_STRING
			BL PutStringSB
			B   InvalidFirst
			
NextPrompt
           
            MOVS R0, #0x0D    ; (CR) moves cursor to new line
			BL PutChar
			MOVS R0, #0x0A    ; (LF)line feed to terminal screen
			BL PutChar
Again2
			LDR R0,=Prompt2
			MOVS R1,#MAX_STRING
			BL PutStringSB
InvalidSecond			
			LSRS R1, R5, #5 ; R1 is the input for Get Hex Int Multi, it indicates that we are using 4 words
			LDR R0, =SecondHex
			BL GetHexIntMulti
			
		    BCC DoneGettingBothNumber
			MOVS R0, #0x0D    ; (CR) moves cursor to new line
			BL PutChar
			MOVS R0, #0x0A    ; (LF)line feed to terminal screen
			BL PutChar
			LDR R0,=Faliure
			MOVS R1,#MAX_STRING
			BL PutStringSB
			B   InvalidSecond
			
DoneGettingBothNumber
           LDR R0, =AdditionHex ; Base address of where to store the addition
           LDR R1, =FirstHex    ; First user hex
		   LDR R2, =SecondHex   ; Second user hex
		   LSRS R3, R5, #5      ; Used to Get the Number of Words
           BL AddIntMultiU      ; Add the two numbers
		   BCC NoOverflow       ; Check if the Addition overflowed or not
		   MOVS R0, #0x0D    ; (CR) moves cursor to new line
		   BL PutChar
		   MOVS R0, #0x0A    ; (LF)line feed to terminal screen
		   BL PutChar
		   LDR R0, =SumPrompt
		   MOVS R1,#MAX_STRING
		   BL PutStringSB
		   LDR R0,=Overflow
		   BL PutStringSB
		   B Again1
NoOverflow
           MOVS R0, #0x0D    ; (CR) moves cursor to new line
		   BL PutChar
		   MOVS R0, #0x0A    ; (LF)line feed to terminal screen
		   BL PutChar
		   LDR R0, =SumPrompt   ; Loading the sum prompt
		   MOVS R1, #MAX_STRING 
		   BL PutStringSB
		   
		   MOVS R5, #NUMBER_OF_BITS; NEED the Counter 32 Hex digits - 128 /4 or NUMBER_OF /4
		   LDR  R0,=AdditionHex ; Loading the value added
		   LSRS R1, R5, #6      ; Used to Get the Number of Words    
           BL PutHexIntMulti

	 	   B  Again1	        ; Storing the New words
;>>>>>   end main program code <<<<<
            B       .
            ENDP
				
;>>>>> begin subroutine code <<<<<
AddIntMultiU   PROC {R0-R14}
;**********************************************************
; R2, and R1 has the address of R3-words in memory and Store the Results in the address (in R0)
; C bit is clear if succesfull C bit is set if the subroutine failed
; Input: R0 - Address to Store: R1 - The word(s) start Address : R2 - The word(s) Start Address
; Output: N/A
; Modify: APSR
; All other registers remain unchanged on return
;**********************************************************
    ; R0 the base address to store the values
	; R1 the base address of the first value
	; R2 the base address of the second value
	; R3 the Number of words
	; R4 the counter to load new values
	; R5 the value of the first  word (+R4 position)
	; R6 the value of the second word (+R4 position)
	; R7 Saves the C bit and loads the C bit
    PUSH {R0-R7}
    
    MRS  	R5, APSR   		   ;CLEAR THE C FLAG
	MOVS 	R6,#0x20
	LSLS 	R6,R6,#24
	BICS 	R5,R5,R6
	MSR  	APSR, R5           ; CLEAR THE C FLAG
	
    MRS     R7,APSR            ; LOAD the C bit       
	
	MOVS R4,#0      ; Intialize the Counter (used to displacement addressing)
	; Need to change it LDRB and STRB then go word by word and ADCS
	; Incrementer needs to go word by word in the array
	
AA  ; What needs to happen is using word addressing and Using word load and storing instead

    LDR R5,[R1,R4]     ; loads the first word value with  address from R2         
    LDR R6,[R2,R4]     ; loads the first word value with  address from R3
	MSR  APSR,R7       ; LOAD the C bit
    ADCS R5,R5,R6      ; after adding the words using ADCS
	MRS  R7, APSR      ; Saves the C bit
	STR  R5,[R0,R4]    ; store the new value                                       
    ADDS R4,R4,#4      ; Increment the value R4 by 8
	SUBS R3,#1 
	CMP  R3,#0       ; do while N > 0
	BGT AA           ; AA being Add Again when R4 > 0
	
	MSR APSR,R7
	BCS OverFlowSet ; After done adding check if over flow to determine whether to clear or set the c bit
    MRS  	R5, APSR   		   ;CLEAR THE C FLAG
	MOVS 	R6,#0x20
	LSLS 	R6,R6,#24
	BICS 	R5,R5,R6
	MSR  	APSR, R5           ; CLEAR THE C FLAG
	B SkipToEnd
OverFlowSet 
	MRS 	R3, APSR; SETING THE C FLAG
	MOVS 	R4, #0x20			          
	LSLS 	R4,R4, #24
	ORRS 	R3,R3,R4
	MSR  	APSR, R3; SETTING THE C FLAG
SkipToEnd           ; This branch is used when the C bit is cleared
    POP {R0-R7}
	BX LR
	ENDP



GetHexIntMulti   PROC {R0-R14}
;**********************************************************
; Ask the User to input a character and Converts them
; Input: R0 - Address to Store of &Num64: R1 - The Number of Words
; Output: N/A
; Modify: APSR
; All other registers remain unchanged on return
;**********************************************************    

    ; R0 Address of the String
	; R1 the N words
	; R2 copy of the Address of the String used for packing
	; R3 Copy of N
	; R5 flag to see if the String is Valid - If the right R5 = 0, if wrong R5 = 1
	
    PUSH{R0-R7,LR}
;****Calling the GETSTRING SB******************************
	; Multiply R1 by 8 and then add 1
	MOVS R5,#0      ; Used as a flag
	MOVS R3,R1      ; Used as a counter for the mutiplication
	MOVS R1, #0     ; Store N*8+1
	MOVS R2,#0      ; Used as a counter
	LDR R6, =Tempt  ; Tempt String
multN	
    ADDS R1,R1,#8   ; Adds 8, should do this N times (n = value of R1)
	SUBS R3,R3,#1   ; Decrement the counter
    CMP  R3,#0      ; Add 8 if R3 >0
    BHI  multN	    ; Add 8 if R3 > 0
	ADDS R1,R1,#1   ; To have 8*N +1 (8*N calculated above)
    ; Multiply R1 by 8 and then add 1	
	                ; Input R0 - the Characters address to store
					; Input R1 - The Length it can Operate
					; Output To terminal and Store the Character
    BL GetStringSB  ; Ask for User to type out the N words of digits 	                 

;****Convert ASCII codes to hex digit values***************
keepConverting
    LDRB R3,[R0,R2]     ; Look at the Address of the value in memory, IS IT LDR OR LDRB
	CMP  R3, #0          ;   Used to check if value is NULL
	BLS  dontSub 
    CMP  R3, #0x41	     ; Need to Check if the ASCII is 41 >= , then the value needs to be subs by 31
	BHS  asciiIsALetter
    SUBS R3,R3,#0x30     ; subtract every value by 30 to CONVERT the Value from ascii to hex decimal decimal till the end of the null pointer
	CMP  R3, #0x09       ; From CMP R3, 09 to skipp it make sure we have the right value
	BLE  nextCheck       ; if R3 < 09 then check if it's > 00 
	ADDS R5,#1          
	B    skipp
nextCheck
    CMP  R3, #0x00
	BGE  skipp
	ADDS R5,#1
skipp
	B skipAscii
asciiIsALetter
    SUBS R3,R3,#0x37   ; ONLY perform this subtraction if the ascii character is A,B,C,D,E,F as the Hex a decimal is postion differently
	; CHECKING IT IT'S VALID
	CMP  R3,#0xF   ; CHECK IF R3 <= F and R3 >= A
	BLS  ValidHex
    ADDS R5,#1
ValidHex
    CMP  R3,#0x0A
	BGE  ValidHex2
	ADDS R5, #1
ValidHex2
skipAscii
    STRB R3,[R0,R2]  ; Store the new value
	ADDS R2,#1
	B keepConverting
dontSub

; Using R1 as the pointer towards the end of memory ()
    PUSH {R0-R3}
;Swapin the Address in R0 with temp to help the bellow method
    SUBS R1,  #1
	LDR  R2,  =Tempt
KeepSwapingValue1
	LDR R3, [R0,R1]  ; Load the value in Temp
	STR R3, [R2,R1]  ; Load the Value into Num64
	SUBS R1,#4
    CMP  R1, #0       ; if ( Number of words ) greater than or equal #zero    
	BGE KeepSwapingValue1
	POP {R0-R3}


;****Pack hex digit values into 4 * n byte values of Num64 in little endian order
    PUSH {R0-R7}
	; Store into tempt address
	SUBS R1, #1     ; Remove the +1 used for the GetStringSB
    MOVS R2, R1     ; Used to calculate the number of words
	LSRS R2, #3
	MOVS R6, R2     ; Hold the number of times we number of times we need to shift
	LSLS R6, #1     ; Which should be the number 8
	SUBS R1, #1     ; Used to start at the end of memory
	MOVS R7, #0     ; Will hold the temp word
	SUBS R1, #7     ; Used to start at a certain word
keepStoringReset
	MOVS R5, #0     ; Counter Value
	MOVS R7, #0     ; Set r7 = 0
	
keepStoring
    LDRB R3,[R0,R1] ; Get the Value at the end       (V1)        - DOUBLE CHECK IF IT'  S STORING NULL 
	ADDS R1,#1      ; Used to get V2
    LDRB R4,[R0,R1] ; Get the Value at the end - 1   (V2)
    LSLS R3, #4     ; Shift the V2 by 1 left 
    ADDS R4,R4,R3   ; Add the V2 to V1            - This enables V2 = A0 (0A before shift) and V2 0D to add together to form (AD)
	ADDS R1,#1      ; Decrements to get the Next V1 value
	LSLS R7 ,#8   ; Shift by two hex to make room
    ADDS R7,R4,R7 ; ADD WITH R4 to get the word
    ADDS R5,#1    ; keep doing this till 4 time
	CMP  R5,#4    ; R5 < number of times need to shift (R6)
    BLT  keepStoring
	
	PUSH {R2}
	; This is how we handle addressiong with a while loop
	MOVS R1,#0
doItAgain
	SUBS R2,#1
	CMP  R2,#0 ; if r2 > 0
	BLE  dontRunIF
	ADDS R1, #4   ; Adds 4 for each WORD (in R2)
	CMP  R2, #1
	BGT doItAgain
	B skiptheZero
dontRunIF
	MOVS R1,#0
skiptheZero 
    POP  {R2}
	
	PUSH {R0}        ; Used to store into Tempt so we can get the new 
    LDR  R0, =Tempt     ; words without erasing memory to store what we packaged
	STR  R7,[R0,R1] 
	POP{R0}
	
	PUSH {R2}
	; This is how we handle addressiong with a while loop
doItAgain2
	SUBS R2,#1
	CMP  R2,#0 ; if r2 > 0
	BLE  dontRunIF1
	SUBS R1, #4   ; Adds 4 for each WORD (in R2)
	CMP  R2, #1
	BGT doItAgain2
	B skiptheZero2
dontRunIF1
	MOVS R1,#0
skiptheZero2 
    POP  {R2}
	
	SUBS R2,#1
	CMP R2, #0            ; IF R2 = 0 then we stop the code
	BEQ STOPSTORING
	B keepStoringReset
STOPSTORING	
    POP {R0-R7}

;Swaping Temp Address with the Address in R0
    LSRS R1, #1
	SUBS R1,#4
	LDR R2,  =Tempt
	;MOVS R6, #8     ; USED TO ROTATE
KeepSwapingValue
	LDR R3, [R2,R1]  ; Load the value in Temp
	;RORS R3,R3,R6   ; ROTATING THE BITS
	STM R0!,{R3}     ; Load the Value into Num64
	SUBS R1,#4
    CMP R1, #0       ; if ( Number of words ) greater than or equal #zero    with 128 bits should run 2 times
	BGE KeepSwapingValue
	
;****Pack hex digit values into 4 ? n byte values of Num64 in little endian order
    CMP R5, #0
	BNE itsWrong
;Clear the C bit
    MRS  	R3, APSR   		   ;CLEAR THE C FLAG
	MOVS 	R4,#0x20
	LSLS 	R4,R4,#24
	BICS 	R3,R3,R4
	MSR  	APSR, R3           ; CLEAR THE C FLAG
    B itsRight
itsWrong
;Set the C bit
    MRS 	R3, APSR; SETING THE C FLAG
	MOVS 	R4, #0x20			          
	LSLS 	R4,R4, #24
	ORRS 	R3,R3,R4
	MSR  	APSR, R3; SETTING THE C FLAG
itsRight
    POP{R0-R7,PC}
    BX LR 
    ENDP


PutHexIntMulti   PROC {R0-R14}
;**********************************************************
; This will inpack the hexdecimal number and print out the 
; Input: R0 - Address to Store of &Num64: R1 - The Number of Words
; Output: N/A
; Modify: APSR
; All other registers remain unchanged on return
;**********************************************************   
    PUSH {R0-R2,LR}
    MOVS R2,R0
	LSLS R1,#3 ; Starts at the end of memory
	SUBS R1,#4 ; Used for intial memory 
PrintAgain
	LDR R0, [R2,R1] ;Loads the word
	BL PutNumHex
	SUBS R1, #4
	CMP  R1, #0      ; if R1 >= 0 loop
	BGE PrintAgain
	POP {R0-R2,PC}
    BX LR 
    ENDP


GetStringSB     PROC     {R1-R14}
;*****************************************************************
; With R0 as the address to the string in memory, this subroutine will take user input and 
; Store it up to R1-1 Length of Characters Until the user presses enter, then afterwards it will return
; the address of the string
; Input: R0, R1
; Ouput: 
; Modify: R0, PSR
;*****************************************************************	            
            PUSH {R0-R4,LR}
			MOVS R2,R0            ; Move the base index of the string to R2
			MOVS R3, #0           ; the counter to insure we don't go Past R1-1 or MAX_STRING -1 
 			MOVS R4, #0x0D        ; Stores the enter key
			SUBS R1, #1           ; to insure that MAX_STRING-1 is the limit
Loop        BL GetChar            ;Gets the Character from the user
            CMP R4, R0            ;Checks if the user pressed "enter"
			BEQ ENDTHIS
            CMP R1, R3            ; Checks if too many characters are typed
			BLS StopStoringChar  
			STRB R0,[R2,R3]       ; Stores given character from the user(R0) into the address of the string
			BL PutChar            ; Puts the Character on    
			ADDS R3,R3,#1         ; Increments 
StopStoringChar B Loop
ENDTHIS                           ; After this the program, in main should continues to ask the user for the next instruction 
            MOVS R4, #0        ; This sets the null at the end of a string, so when reading the string the other subrotine will stop      
            STRB R4,[R2,R3]
            POP {R0-R4,PC}
            BX LR
			ENDP

PutNumHex     PROC {R1-R14}
;**********************************************************
; Print to the terminal screen the Hexadecimal Representation
; Of the Unsigned Value in R0
; Input: R0:  The unsigned word value 
; Output: N/A Terminal display
; Modify: APSR
; All other registers remain unchanged on return
;**********************************************************
            PUSH {R0-R5,LR}
			
			MOVS R2, R0            ; Hold the value
			MOVS R3, #0            ; Counter
			MOVS R5, R0            ; Constant Hex value
			MOVS R4, #28           ; Intial Shift amount
			
again1		MOVS R2,R5             ; restart R2
			LSRS R2,R2,R4          ; Shift Value by 28-n
			SUBS R4,R4,#4 
			LDR R1,=0x0000000F     ; Load the Register with mask
			
			
            MOVS R0,R2             ; Hold to Value to use R0 as an input
   		    ANDS R0,R0,R1          ; AND THE VALUES 
			CMP R0, #10            ; Checks before adding '0' s ascii value
			BHS	convert
			ADDS R0,R0,#'0'        ; Change the Hex value intp ASCII
			B 	skip
convert
			SUBS R0,R0,#10
			ADDS R0,R0,#'A'
skip    	BL  PutChar            ; PRINT
			ADDS R3,R3,#1          ; Increments the counter
			CMP R3,#8              ; LOOP 
			BLO again1             ; checks if the 
			
			POP {R0-R5,PC}
            BX LR
            ENDP
				
		
GetChar    PROC		{R1-R14}
;*****************************************************************
; This Program will read a single character from the terminal keyboard into R0
; Input: from Terminal Screen
; Ouput: R0 
;*****************************************************************			
          PUSH	{R1-R3}							

		  LDR 	R1,=UART0_BASE
		  MOVS 	R2,#UART0_S1_RDRF_MASK
PollRx 	  LDRB 	R3,[R1,#UART0_S1_OFFSET]
		  ANDS 	R3,R3,R2
		  BEQ 	PollRx
		  LDRB 	R0,[R1,#UART0_D_OFFSET]
		  POP		{R1-R3}
		  BX		LR
		  ENDP
			  
PutChar    PROC		{R0-R14}
;*****************************************************************
; This Program will Display the single character from R0 to the terminal screen
; Input: R0
; Ouput: Digital on the screen
;*****************************************************************
         PUSH	{R1-R3}							
		 LDR	R1,=UART0_BASE
		 MOVS 	R2,#UART0_S1_TDRE_MASK
PollTx 	 LDRB 	R3,[R1,#UART0_S1_OFFSET]
		 ANDS 	R3,R3,R2
		 BEQ 	PollTx
		 STRB 	R0,[R1,#UART0_D_OFFSET]
		 
		 POP		{R1-R3}
		 BX		LR
		 ENDP

PutStringSB     PROC     {R0-R14}
;*****************************************************************
; With R0 as the address to the string in memory, this subroutine will take the string stored in R0
; And read it out to the terminal until it read a null pointer
; Input: R0, R1
; Ouput: R0
; Modify: R0, PSR
;*****************************************************************	            	
           PUSH {R0-R5, LR}
		   MOVS R4, R0          ; Store the base address of the string
		   MOVS  R2, #0          ; Used to check if null	 
           MOVS R3, #0          ; Intializing the Counter	
           MOVS R5, #'&'        ; Used to check	&	   
readLoop   LDRB R0,[R4,R3]      ; Checks if the next variable is NULL
		   CMP R0, R2           ; ^
		   BEQ StopReading
		   CMP R0, R5
		   BEQ StopReading      ; Stops reading if it's &
		   BL   PutChar         ; Print into the terminal the Character from [R2, R3] 
		   ADDS R3,R3, #1       ; Increments the Pointer
		   B readLoop
StopReading
		   POP {R0-R5, PC}
           BX LR
           ENDP

			 
Init_UART0_Polling   PROC   {R0-R14}
;*****************************************************************
; This Program will initialize the UART to be used in for PutChar and GetChar
; Input: 
; Ouput:
;*****************************************************************
				 PUSH	{R1-R3}
				 LDR 	R1,=SIM_SOPT2
				 LDR 	R2,=SIM_SOPT2_UART0SRC_MASK
				 LDR 	R3,[R1,#0]
				 BICS 	R3,R3,R2
				 LDR 	R2,=SIM_SOPT2_UART0SRC_MCGFLLCLK
				 ORRS 	R3,R3,R2
				 STR 	R3,[R1,#0]
				 LDR 	R1,=SIM_SOPT5
				 LDR 	R2,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
				 LDR 	R3,[R1,#0]
				 BICS 	R3,R3,R2
				 STR 	R3,[R1,#0]
				 LDR 	R1,=SIM_SCGC4
				 LDR 	R2,=SIM_SCGC4_UART0_MASK
				 LDR 	R3,[R1,#0]
				 ORRS 	R3,R3,R2
				 STR 	R3,[R1,#0]
				 LDR 	R1,=SIM_SCGC5
				 LDR 	R2,=SIM_SCGC5_PORTB_MASK
				 LDR 	R3,[R1,#0]
				 ORRS 	R3,R3,R2
				 STR 	R3,[R1,#0]
				 LDR 	R1,=PORTB_PCR2
				 LDR 	R2,=PORT_PCR_SET_PTB2_UART0_RX
				 STR 	R2,[R1,#0]
				 LDR 	R1,=PORTB_PCR1
				 LDR 	R2,=PORT_PCR_SET_PTB1_UART0_TX
				 STR 	R2,[R1,#0]
				 LDR 	R1,=UART0_BASE
				 MOVS 	R2,#UART0_C2_T_R
				 LDRB 	R3,[R1,#UART0_C2_OFFSET]
				 BICS 	R3,R3,R2
				 STRB 	R3,[R1,#UART0_C2_OFFSET]
				 MOVS 	R2,#UART0_BDH_9600
				 STRB 	R2,[R1,#UART0_BDH_OFFSET]
				 MOVS 	R2,#UART0_BDL_9600
				 STRB 	R2,[R1,#UART0_BDL_OFFSET]
				 MOVS 	R2,#UART0_C1_8N1
				 STRB 	R2,[R1,#UART0_C1_OFFSET]
				 MOVS 	R2,#UART0_C3_NO_TXINV
				 STRB 	R2,[R1,#UART0_C3_OFFSET]
				 MOVS 	R2,#UART0_C4_NO_MATCH_OSR_16
				 STRB 	R2,[R1,#UART0_C4_OFFSET]
				 MOVS 	R2,#UART0_C5_NO_DMA_SSR_SYNC
				 STRB 	R2,[R1,#UART0_C5_OFFSET]
				 MOVS 	R2,#UART0_S1_CLEAR_FLAGS
				 STRB 	R2,[R1,#UART0_S1_OFFSET]
				 MOVS 	R2,#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
				 STRB 	R2,[R1,#UART0_S2_OFFSET] 
				 MOVS 	R2,#UART0_C2_T_R
				 STRB 	R2,[R1,#UART0_C2_OFFSET] 
				 POP	{R1-R3}
				 BX		LR
				 ENDP
;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors
            EXPORT  __Vectors_End
            EXPORT  __Vectors_Size
            IMPORT  __initial_sp
            IMPORT  Dummy_Handler
            IMPORT  HardFault_Handler
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;01:reset vector
            DCD    Dummy_Handler      ;02:NMI
            DCD    HardFault_Handler  ;03:hard fault
            DCD    Dummy_Handler      ;04:(reserved)
            DCD    Dummy_Handler      ;05:(reserved)
            DCD    Dummy_Handler      ;06:(reserved)
            DCD    Dummy_Handler      ;07:(reserved)
            DCD    Dummy_Handler      ;08:(reserved)
            DCD    Dummy_Handler      ;09:(reserved)
            DCD    Dummy_Handler      ;10:(reserved)
            DCD    Dummy_Handler      ;11:SVCall (supervisor call)
            DCD    Dummy_Handler      ;12:(reserved)
            DCD    Dummy_Handler      ;13:(reserved)
            DCD    Dummy_Handler      ;14:PendSV (PendableSrvReq)
                                      ;   pendable request 
                                      ;   for system service)
            DCD    Dummy_Handler      ;15:SysTick (system tick timer)
            DCD    Dummy_Handler      ;16:DMA channel 0 transfer 
                                      ;   complete/error
            DCD    Dummy_Handler      ;17:DMA channel 1 transfer
                                      ;   complete/error
            DCD    Dummy_Handler      ;18:DMA channel 2 transfer
                                      ;   complete/error
            DCD    Dummy_Handler      ;19:DMA channel 3 transfer
                                      ;   complete/error
            DCD    Dummy_Handler      ;20:(reserved)
            DCD    Dummy_Handler      ;21:FTFA command complete/
                                      ;   read collision
            DCD    Dummy_Handler      ;22:low-voltage detect;
                                      ;   low-voltage warning
            DCD    Dummy_Handler      ;23:low leakage wakeup
            DCD    Dummy_Handler      ;24:I2C0
            DCD    Dummy_Handler      ;25:(reserved)
            DCD    Dummy_Handler      ;26:SPI0
            DCD    Dummy_Handler      ;27:(reserved)
            DCD    Dummy_Handler      ;28:UART0 (status; error)
            DCD    Dummy_Handler      ;29:(reserved)
            DCD    Dummy_Handler      ;30:(reserved)
            DCD    Dummy_Handler      ;31:ADC0
            DCD    Dummy_Handler      ;32:CMP0
            DCD    Dummy_Handler      ;33:TPM0
            DCD    Dummy_Handler      ;34:TPM1
            DCD    Dummy_Handler      ;35:(reserved)
            DCD    Dummy_Handler      ;36:RTC (alarm)
            DCD    Dummy_Handler      ;37:RTC (seconds)
            DCD    Dummy_Handler      ;38:PIT
            DCD    Dummy_Handler      ;39:(reserved)
            DCD    Dummy_Handler      ;40:(reserved)
            DCD    Dummy_Handler      ;41:DAC0
            DCD    Dummy_Handler      ;42:TSI0
            DCD    Dummy_Handler      ;43:MCG
            DCD    Dummy_Handler      ;44:LPTMR0
            DCD    Dummy_Handler      ;45:(reserved)
            DCD    Dummy_Handler      ;46:PORTA
            DCD    Dummy_Handler      ;47:PORTB
__Vectors_End
__Vectors_Size  EQU     __Vectors_End - __Vectors
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
Prompt1 DCB " Enter first 128-bit hex number:  0x\0" 
Prompt2 DCB "Enter 128-bit hex number to add:  0x\0"
Faliure DCB "      Invalid number--try again:  0x\0"
Overflow DCB "OVERFLOW \0"
SumPrompt DCB "                            Sum:  0x\0"
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
    ALIGN
Num64     SPACE   (NUMBER_OF_BITS / 3)     ; Stores the 64 value
	ALIGN
FirstHex  SPACE   (NUMBER_OF_BITS / 4)     ; Reserves 16 
	ALIGN
SecondHex SPACE   (NUMBER_OF_BITS / 4)     ; Reserves 16
	ALIGN
AdditionHex SPACE   (NUMBER_OF_BITS / 4)     ; Reserves 16
	ALIGN
Tempt     SPACE   (NUMBER_OF_BITS / 4)     ; Reserves 16
	ALIGN
;>>>>>   end variables here <<<<<
            ALIGN
            END