	.syntax unified
	.cpu cortex-m3
	.fpu softvfp
	.eabi_attribute 20, 1
	.eabi_attribute 21, 1
	.eabi_attribute 23, 3
	.eabi_attribute 24, 1
	.eabi_attribute 25, 1
	.eabi_attribute 26, 1
	.eabi_attribute 30, 2
	.eabi_attribute 18, 4
	.thumb
	.file	"robot_comp.c"
	.section	.text.digital_capture_1P,"ax",%progbits
	.align	2
	.global	digital_capture_1P
	.thumb
	.thumb_func
	.type	digital_capture_1P, %function
digital_capture_1P:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	movw	r3, #:lower16:.LANCHOR0
	movt	r3, #:upper16:.LANCHOR0
	ldr	r1, [r3, #8]
	mov	r2, #3072
	adds	r1, r1, #1
	str	r1, [r3, #8]
	movt	r2, 16384
	ldr	r2, [r2, #36]
	str	r2, [r3, #0]
	bx	lr
	.size	digital_capture_1P, .-digital_capture_1P
	.section	.text.digital_capture_1N,"ax",%progbits
	.align	2
	.global	digital_capture_1N
	.thumb
	.thumb_func
	.type	digital_capture_1N, %function
digital_capture_1N:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	movw	r3, #:lower16:.LANCHOR0
	movt	r3, #:upper16:.LANCHOR0
	ldr	r1, [r3, #12]
	mov	r2, #3072
	adds	r1, r1, #1
	str	r1, [r3, #12]
	movt	r2, 16384
	ldr	r2, [r2, #36]
	str	r2, [r3, #4]
	bx	lr
	.size	digital_capture_1N, .-digital_capture_1N
	.section	.text.digital_capture_2P,"ax",%progbits
	.align	2
	.global	digital_capture_2P
	.thumb
	.thumb_func
	.type	digital_capture_2P, %function
digital_capture_2P:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	movw	r3, #:lower16:.LANCHOR0
	movt	r3, #:upper16:.LANCHOR0
	ldr	r1, [r3, #16]
	mov	r2, #3072
	adds	r1, r1, #1
	str	r1, [r3, #16]
	movt	r2, 16384
	ldr	r2, [r2, #36]
	str	r2, [r3, #0]
	bx	lr
	.size	digital_capture_2P, .-digital_capture_2P
	.section	.text.digital_capture_2N,"ax",%progbits
	.align	2
	.global	digital_capture_2N
	.thumb
	.thumb_func
	.type	digital_capture_2N, %function
digital_capture_2N:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	movw	r3, #:lower16:.LANCHOR0
	movt	r3, #:upper16:.LANCHOR0
	ldr	r1, [r3, #20]
	mov	r2, #3072
	adds	r1, r1, #1
	str	r1, [r3, #20]
	movt	r2, 16384
	ldr	r2, [r2, #36]
	str	r2, [r3, #4]
	bx	lr
	.size	digital_capture_2N, .-digital_capture_2N
	.section	.text.digital_capture_0P,"ax",%progbits
	.align	2
	.global	digital_capture_0P
	.thumb
	.thumb_func
	.type	digital_capture_0P, %function
digital_capture_0P:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	movw	r3, #:lower16:.LANCHOR0
	movt	r3, #:upper16:.LANCHOR0
	ldr	r1, [r3, #24]
	mov	r2, #3072
	adds	r1, r1, #1
	str	r1, [r3, #24]
	movt	r2, 16384
	ldr	r2, [r2, #36]
	str	r2, [r3, #0]
	bx	lr
	.size	digital_capture_0P, .-digital_capture_0P
	.section	.text.digital_capture_0N,"ax",%progbits
	.align	2
	.global	digital_capture_0N
	.thumb
	.thumb_func
	.type	digital_capture_0N, %function
digital_capture_0N:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	movw	r3, #:lower16:.LANCHOR0
	movt	r3, #:upper16:.LANCHOR0
	ldr	r1, [r3, #28]
	mov	r2, #3072
	adds	r1, r1, #1
	str	r1, [r3, #28]
	movt	r2, 16384
	ldr	r2, [r2, #36]
	str	r2, [r3, #4]
	bx	lr
	.size	digital_capture_0N, .-digital_capture_0N
	.section	.text.digital_capture_10P,"ax",%progbits
	.align	2
	.global	digital_capture_10P
	.thumb
	.thumb_func
	.type	digital_capture_10P, %function
digital_capture_10P:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L12
	mov	r1, #2048
	ldr	r0, [r3, #40]
	ldr	r2, [r3, #64]
	adds	r0, r0, #1
	str	r0, [r3, #40]
	movt	r1, 16384
	ldr	r1, [r1, #36]
	uxth	r0, r2
	rsb	ip, r1, r0
	cmn	ip, #32768
	eor	r2, r0, r2
	ble	.L11
	cmp	ip, #32768
	it	ge
	addge	r2, r2, #65536
.L9:
	orrs	r2, r2, r1
	str	r2, [r3, #64]
	str	r2, [r3, #32]
	bx	lr
.L11:
	sub	r2, r2, #65536
	movw	ip, #65535
	eor	r2, r2, ip
	b	.L9
.L13:
	.align	2
.L12:
	.word	.LANCHOR0
	.size	digital_capture_10P, .-digital_capture_10P
	.section	.text.digital_capture_10N,"ax",%progbits
	.align	2
	.global	digital_capture_10N
	.thumb
	.thumb_func
	.type	digital_capture_10N, %function
digital_capture_10N:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L18
	mov	r1, #2048
	ldr	r0, [r3, #44]
	ldr	r2, [r3, #64]
	adds	r0, r0, #1
	str	r0, [r3, #44]
	movt	r1, 16384
	ldr	r1, [r1, #36]
	uxth	r0, r2
	rsb	ip, r1, r0
	cmn	ip, #32768
	eor	r2, r0, r2
	ble	.L17
	cmp	ip, #32768
	it	ge
	addge	r2, r2, #65536
.L16:
	orrs	r2, r2, r1
	str	r2, [r3, #64]
	str	r2, [r3, #36]
	bx	lr
.L17:
	sub	r2, r2, #65536
	movw	ip, #65535
	eor	r2, r2, ip
	b	.L16
.L19:
	.align	2
.L18:
	.word	.LANCHOR0
	.size	digital_capture_10N, .-digital_capture_10N
	.section	.text.digital_capture_11P,"ax",%progbits
	.align	2
	.global	digital_capture_11P
	.thumb
	.thumb_func
	.type	digital_capture_11P, %function
digital_capture_11P:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L24
	mov	r1, #2048
	ldr	r0, [r3, #48]
	ldr	r2, [r3, #64]
	adds	r0, r0, #1
	str	r0, [r3, #48]
	movt	r1, 16384
	ldr	r1, [r1, #36]
	uxth	r0, r2
	rsb	ip, r1, r0
	cmn	ip, #32768
	eor	r2, r0, r2
	ble	.L23
	cmp	ip, #32768
	it	ge
	addge	r2, r2, #65536
.L22:
	orrs	r2, r2, r1
	str	r2, [r3, #64]
	str	r2, [r3, #32]
	bx	lr
.L23:
	sub	r2, r2, #65536
	movw	ip, #65535
	eor	r2, r2, ip
	b	.L22
.L25:
	.align	2
.L24:
	.word	.LANCHOR0
	.size	digital_capture_11P, .-digital_capture_11P
	.section	.text.digital_capture_11N,"ax",%progbits
	.align	2
	.global	digital_capture_11N
	.thumb
	.thumb_func
	.type	digital_capture_11N, %function
digital_capture_11N:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L30
	mov	r1, #2048
	ldr	r0, [r3, #52]
	ldr	r2, [r3, #64]
	adds	r0, r0, #1
	str	r0, [r3, #52]
	movt	r1, 16384
	ldr	r1, [r1, #36]
	uxth	r0, r2
	rsb	ip, r1, r0
	cmn	ip, #32768
	eor	r2, r0, r2
	ble	.L29
	cmp	ip, #32768
	it	ge
	addge	r2, r2, #65536
.L28:
	orrs	r2, r2, r1
	str	r2, [r3, #64]
	str	r2, [r3, #36]
	bx	lr
.L29:
	sub	r2, r2, #65536
	movw	ip, #65535
	eor	r2, r2, ip
	b	.L28
.L31:
	.align	2
.L30:
	.word	.LANCHOR0
	.size	digital_capture_11N, .-digital_capture_11N
	.section	.text.digital_capture_9P,"ax",%progbits
	.align	2
	.global	digital_capture_9P
	.thumb
	.thumb_func
	.type	digital_capture_9P, %function
digital_capture_9P:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L36
	mov	r1, #2048
	ldr	r0, [r3, #56]
	ldr	r2, [r3, #64]
	adds	r0, r0, #1
	str	r0, [r3, #56]
	movt	r1, 16384
	ldr	r1, [r1, #36]
	uxth	r0, r2
	rsb	ip, r1, r0
	cmn	ip, #32768
	eor	r2, r0, r2
	ble	.L35
	cmp	ip, #32768
	it	ge
	addge	r2, r2, #65536
.L34:
	orrs	r2, r2, r1
	str	r2, [r3, #64]
	str	r2, [r3, #32]
	bx	lr
.L35:
	sub	r2, r2, #65536
	movw	ip, #65535
	eor	r2, r2, ip
	b	.L34
.L37:
	.align	2
.L36:
	.word	.LANCHOR0
	.size	digital_capture_9P, .-digital_capture_9P
	.section	.text.digital_capture_9N,"ax",%progbits
	.align	2
	.global	digital_capture_9N
	.thumb
	.thumb_func
	.type	digital_capture_9N, %function
digital_capture_9N:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L42
	mov	r1, #2048
	ldr	r0, [r3, #60]
	ldr	r2, [r3, #64]
	adds	r0, r0, #1
	str	r0, [r3, #60]
	movt	r1, 16384
	ldr	r1, [r1, #36]
	uxth	r0, r2
	rsb	ip, r1, r0
	cmn	ip, #32768
	eor	r2, r0, r2
	ble	.L41
	cmp	ip, #32768
	it	ge
	addge	r2, r2, #65536
.L40:
	orrs	r2, r2, r1
	str	r2, [r3, #64]
	str	r2, [r3, #36]
	bx	lr
.L41:
	sub	r2, r2, #65536
	movw	ip, #65535
	eor	r2, r2, ip
	b	.L40
.L43:
	.align	2
.L42:
	.word	.LANCHOR0
	.size	digital_capture_9N, .-digital_capture_9N
	.section	.text.digital_capture_5P,"ax",%progbits
	.align	2
	.global	digital_capture_5P
	.thumb
	.thumb_func
	.type	digital_capture_5P, %function
digital_capture_5P:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L48
	mov	r1, #1024
	ldr	r0, [r3, #68]
	ldr	r2, [r3, #76]
	adds	r0, r0, #1
	str	r0, [r3, #68]
	movt	r1, 16385
	ldr	r1, [r1, #36]
	uxth	r0, r2
	rsb	ip, r1, r0
	cmn	ip, #32768
	eor	r2, r0, r2
	ble	.L47
	cmp	ip, #32768
	it	ge
	addge	r2, r2, #65536
.L46:
	orrs	r2, r2, r1
	str	r2, [r3, #76]
	str	r2, [r3, #80]
	bx	lr
.L47:
	sub	r2, r2, #65536
	movw	ip, #65535
	eor	r2, r2, ip
	b	.L46
.L49:
	.align	2
.L48:
	.word	.LANCHOR0
	.size	digital_capture_5P, .-digital_capture_5P
	.section	.text.digital_capture_5N,"ax",%progbits
	.align	2
	.global	digital_capture_5N
	.thumb
	.thumb_func
	.type	digital_capture_5N, %function
digital_capture_5N:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L54
	mov	r1, #1024
	ldr	r0, [r3, #72]
	ldr	r2, [r3, #76]
	adds	r0, r0, #1
	str	r0, [r3, #72]
	movt	r1, 16385
	ldr	r1, [r1, #36]
	uxth	r0, r2
	rsb	ip, r1, r0
	cmn	ip, #32768
	eor	r2, r0, r2
	ble	.L53
	cmp	ip, #32768
	it	ge
	addge	r2, r2, #65536
.L52:
	orrs	r2, r2, r1
	str	r2, [r3, #76]
	str	r2, [r3, #84]
	bx	lr
.L53:
	sub	r2, r2, #65536
	movw	ip, #65535
	eor	r2, r2, ip
	b	.L52
.L55:
	.align	2
.L54:
	.word	.LANCHOR0
	.size	digital_capture_5N, .-digital_capture_5N
	.section	.text.digital_capture_4P,"ax",%progbits
	.align	2
	.global	digital_capture_4P
	.thumb
	.thumb_func
	.type	digital_capture_4P, %function
digital_capture_4P:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L60
	mov	r1, #1024
	ldr	r0, [r3, #88]
	ldr	r2, [r3, #76]
	adds	r0, r0, #1
	str	r0, [r3, #88]
	movt	r1, 16385
	ldr	r1, [r1, #36]
	uxth	r0, r2
	rsb	ip, r1, r0
	cmn	ip, #32768
	eor	r2, r0, r2
	ble	.L59
	cmp	ip, #32768
	it	ge
	addge	r2, r2, #65536
.L58:
	orrs	r2, r2, r1
	str	r2, [r3, #76]
	str	r2, [r3, #96]
	bx	lr
.L59:
	sub	r2, r2, #65536
	movw	ip, #65535
	eor	r2, r2, ip
	b	.L58
.L61:
	.align	2
.L60:
	.word	.LANCHOR0
	.size	digital_capture_4P, .-digital_capture_4P
	.section	.text.digital_capture_4N,"ax",%progbits
	.align	2
	.global	digital_capture_4N
	.thumb
	.thumb_func
	.type	digital_capture_4N, %function
digital_capture_4N:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L66
	mov	r1, #1024
	ldr	r0, [r3, #92]
	ldr	r2, [r3, #76]
	adds	r0, r0, #1
	str	r0, [r3, #92]
	movt	r1, 16385
	ldr	r1, [r1, #36]
	uxth	r0, r2
	rsb	ip, r1, r0
	cmn	ip, #32768
	eor	r2, r0, r2
	ble	.L65
	cmp	ip, #32768
	it	ge
	addge	r2, r2, #65536
.L64:
	orrs	r2, r2, r1
	str	r2, [r3, #76]
	str	r2, [r3, #100]
	bx	lr
.L65:
	sub	r2, r2, #65536
	movw	ip, #65535
	eor	r2, r2, ip
	b	.L64
.L67:
	.align	2
.L66:
	.word	.LANCHOR0
	.size	digital_capture_4N, .-digital_capture_4N
	.section	.text.TIM1_TR_GCOM_TIM11_IRQHandler,"ax",%progbits
	.align	2
	.global	TIM1_TR_GCOM_TIM11_IRQHandler
	.thumb
	.thumb_func
	.type	TIM1_TR_GCOM_TIM11_IRQHandler, %function
TIM1_TR_GCOM_TIM11_IRQHandler:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r3, #18432
	movt	r3, 16385
	movw	r2, 65533	@ movhi
	strh	r2, [r3, #16]	@ movhi
	bx	lr
	.size	TIM1_TR_GCOM_TIM11_IRQHandler, .-TIM1_TR_GCOM_TIM11_IRQHandler
	.section	.text.robot_init,"ax",%progbits
	.align	2
	.global	robot_init
	.thumb
	.thumb_func
	.type	robot_init, %function
robot_init:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r3, #1024
	movt	r3, 16386
	ldr	r1, [r3, #0]
	mov	r2, #14336
	bic	r1, r1, #786432
	orr	r1, r1, #524288
	str	r1, [r3, #0]
	ldr	r1, [r3, #36]
	movt	r2, 16386
	bic	r1, r1, #240
	orr	r1, r1, #48
	str	r1, [r3, #36]
	ldr	r0, [r2, #68]
	mov	r3, #18432
	orr	r0, r0, #262144
	str	r0, [r2, #68]
	movt	r3, 16385
	movs	r1, #1
	movw	r2, 1199	@ movhi
	strh	r1, [r3, #24]	@ movhi
	strh	r2, [r3, #40]	@ movhi
	movs	r2, #4
	strh	r2, [r3, #0]	@ movhi
	movs	r2, #2
	strh	r2, [r3, #12]	@ movhi
	movs	r2, #3
	strh	r2, [r3, #32]	@ movhi
	strh	r1, [r3, #20]	@ movhi
	ldrh	r1, [r3, #0]
	mov	r2, #57600
	uxth	r1, r1
	orr	r1, r1, #1
	strh	r1, [r3, #0]	@ movhi
	movt	r2, 57344
	movs	r3, #16
	strb	r3, [r2, #794]
	mov	r3, #67108864
	str	r3, [r2, #0]
	bx	lr
	.size	robot_init, .-robot_init
	.section	.text.robot_fill_packet,"ax",%progbits
	.align	2
	.global	robot_fill_packet
	.thumb
	.thumb_func
	.type	robot_fill_packet, %function
robot_fill_packet:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L80
	mov	r2, #1024
	ldr	r1, [r3, #104]
	movt	r2, 16384
	ldr	r2, [r2, #36]
	uxth	ip, r1
	push	{r4, r5, r6, r7, r8, sl}
	rsb	r4, r2, ip
	cmn	r4, #32768
	eor	r1, ip, r1
	ble	.L77
	cmp	r4, #32768
	it	ge
	addge	r1, r1, #65536
.L72:
	orr	r2, r1, r2
	lsrs	r1, r2, #16
	str	r2, [r3, #104]
	strh	r2, [r0, #62]	@ movhi
	strh	r1, [r0, #64]	@ movhi
	mov	r2, #1073741824
	ldr	r2, [r2, #36]
	ldr	r1, .L80+4
	lsr	ip, r2, #16
	strh	r2, [r0, #66]	@ movhi
	strh	ip, [r0, #68]	@ movhi
	ldrh	sl, [r1, #14]
	ldr	r7, [r3, #108]
	strh	sl, [r0, #90]	@ movhi
	ldrh	sl, [r1, #12]
	mov	r2, #57600
	strh	sl, [r0, #92]	@ movhi
	ldrh	sl, [r1, #10]
	movt	r2, 57344
	strh	sl, [r0, #98]	@ movhi
	ldrh	sl, [r1, #0]
	mov	ip, #256
	strh	sl, [r0, #94]	@ movhi
	ldrh	sl, [r1, #2]
	ldrh	r1, [r1, #6]
	mov	r4, #8388608
	lsr	r8, r7, #16
	mov	r6, #1024
	mov	r5, #3072
	strh	r7, [r0, #70]	@ movhi
	strh	r8, [r0, #72]	@ movhi
	strh	r1, [r0, #100]	@ movhi
	strh	sl, [r0, #96]	@ movhi
	movt	r5, 16384
	str	r4, [r2, #128]
	str	ip, [r2, #132]
	str	r6, [r2, #128]
	ldr	r8, [r5, #36]
	ldr	r7, [r3, #0]
	ldr	r5, [r3, #4]
	mov	r1, #2048
	str	r5, [r0, #28]
	movt	r1, 16386
	str	r8, [r0, #24]
	str	r7, [r0, #32]
	ldr	r7, [r1, #16]
	ldrb	r5, [r0, #42]	@ zero_extendqisi2
	ubfx	r7, r7, #5, #1
	bfi	r5, r7, #0, #1
	strb	r5, [r0, #42]
	ldr	r7, [r1, #16]
	uxtb	r5, r5
	ubfx	r7, r7, #13, #1
	bfi	r5, r7, #1, #1
	strb	r5, [r0, #42]
	ldr	r5, [r1, #16]
	ldrb	r1, [r0, #42]	@ zero_extendqisi2
	ubfx	r5, r5, #4, #1
	bfi	r1, r5, #2, #1
	strb	r1, [r0, #42]
	ldr	r5, [r3, #12]
	mov	r1, #2048
	strb	r5, [r0, #36]
	ldr	r5, [r3, #8]
	movt	r1, 16384
	strb	r5, [r0, #37]
	ldr	r5, [r3, #20]
	strb	r5, [r0, #38]
	ldr	r5, [r3, #16]
	strb	r5, [r0, #39]
	ldr	r5, [r3, #28]
	strb	r5, [r0, #40]
	ldr	r5, [r3, #24]
	strb	r5, [r0, #41]
	ldr	r5, [r3, #64]
	str	r4, [r2, #0]
	str	ip, [r2, #4]
	str	r6, [r2, #0]
	str	r4, [r2, #128]
	str	ip, [r2, #128]
	str	ip, [r2, #132]
	ldr	r1, [r1, #36]
	uxth	r2, r5
	rsb	ip, r1, r2
	eors	r2, r2, r5
	cmn	ip, #32768
	ble	.L78
	cmp	ip, #32768
	it	ge
	addge	r2, r2, #65536
.L74:
	orrs	r2, r2, r1
	str	r2, [r3, #64]
	ldr	r1, [r0, #44]
	strb	r2, [r0, #43]
	and	r1, r1, #-16777216
	orr	r2, r1, r2, lsr #8
	str	r2, [r0, #44]
	ldr	r1, [r0, #52]
	ldr	r2, [r3, #32]
	and	r1, r1, #-16777216
	strb	r2, [r0, #51]
	orr	r2, r1, r2, lsr #8
	str	r2, [r0, #52]
	ldr	ip, [r0, #48]
	ldr	r1, [r3, #36]
	and	ip, ip, #-16777216
	orr	ip, ip, r1, lsr #8
	movs	r2, #0
	str	ip, [r0, #48]
	strb	r1, [r0, #47]
	movt	r2, 16386
	ldr	r1, [r2, #16]
	ldrb	ip, [r0, #61]	@ zero_extendqisi2
	ubfx	r1, r1, #7, #1
	bfi	ip, r1, #0, #1
	mov	r1, #1024
	strb	ip, [r0, #61]
	movt	r1, 16386
	ldr	ip, [r1, #16]
	ldrb	r1, [r0, #61]	@ zero_extendqisi2
	ubfx	ip, ip, #2, #1
	bfi	r1, ip, #1, #1
	strb	r1, [r0, #61]
	ldr	r1, [r2, #16]
	ldrb	r2, [r0, #61]	@ zero_extendqisi2
	ubfx	r1, r1, #11, #1
	bfi	r2, r1, #2, #1
	strb	r2, [r0, #61]
	ldr	r1, [r3, #44]
	mov	r2, #57600
	strb	r1, [r0, #55]
	ldr	r1, [r3, #40]
	movt	r2, 57344
	strb	r1, [r0, #56]
	ldr	ip, [r3, #52]
	mov	r1, #256
	strb	ip, [r0, #57]
	ldr	ip, [r3, #48]
	mov	r4, #1024
	strb	ip, [r0, #58]
	ldr	ip, [r3, #60]
	movt	r4, 16385
	strb	ip, [r0, #59]
	ldr	ip, [r3, #56]
	strb	ip, [r0, #60]
	mov	ip, #8388608
	str	ip, [r2, #0]
	str	r1, [r2, #0]
	ldr	ip, [r3, #76]
	str	r1, [r2, #4]
	str	r1, [r2, #132]
	str	r1, [r2, #132]
	ldr	r1, [r4, #36]
	uxth	r2, ip
	subs	r4, r2, r1
	cmn	r4, #32768
	eor	r2, r2, ip
	ble	.L79
	cmp	r4, #32768
	it	ge
	addge	r2, r2, #65536
.L76:
	orrs	r2, r2, r1
	lsrs	r1, r2, #16
	str	r2, [r3, #76]
	strh	r2, [r0, #78]	@ movhi
	strh	r1, [r0, #80]	@ movhi
	ldr	r1, [r3, #84]
	mov	r2, #1024
	lsr	ip, r1, #16
	strh	ip, [r0, #84]	@ movhi
	strh	r1, [r0, #82]	@ movhi
	ldr	r1, [r3, #100]
	movt	r2, 16386
	lsr	ip, r1, #16
	strh	ip, [r0, #88]	@ movhi
	strh	r1, [r0, #86]	@ movhi
	ldr	r1, [r2, #16]
	ldrb	ip, [r0, #106]	@ zero_extendqisi2
	ubfx	r1, r1, #10, #1
	bfi	ip, r1, #1, #1
	mov	r1, #2048
	strb	ip, [r0, #106]
	movt	r1, 16386
	ldr	ip, [r1, #16]
	ldrb	r1, [r0, #106]	@ zero_extendqisi2
	ubfx	ip, ip, #15, #1
	bfi	r1, ip, #2, #1
	strb	r1, [r0, #106]
	ldr	ip, [r3, #72]
	mov	r1, #57600
	strb	ip, [r0, #102]
	ldr	ip, [r3, #68]
	movt	r1, 57344
	strb	ip, [r0, #103]
	ldr	r4, [r3, #92]
	mov	ip, #256
	strb	r4, [r0, #104]
	ldr	r4, [r3, #88]
	movs	r3, #0
	strb	r4, [r0, #105]
	str	ip, [r1, #4]
	str	ip, [r1, #4]
	ldr	r1, [r2, #16]
	ldrb	r2, [r0, #106]	@ zero_extendqisi2
	ubfx	r1, r1, #8, #1
	bfi	r2, r1, #0, #1
	strb	r2, [r0, #106]
	movt	r3, 16386
	ldr	r2, [r3, #16]
	ldrb	r3, [r0, #106]	@ zero_extendqisi2
	ubfx	r2, r2, #12, #1
	bfi	r3, r2, #3, #1
	strb	r3, [r0, #106]
	pop	{r4, r5, r6, r7, r8, sl}
	bx	lr
.L77:
	sub	r1, r1, #65536
	movw	r4, #65535
	eors	r1, r1, r4
	b	.L72
.L79:
	sub	r2, r2, #65536
	movw	r4, #65535
	eors	r2, r2, r4
	b	.L76
.L78:
	sub	r2, r2, #65536
	movw	ip, #65535
	eor	r2, r2, ip
	b	.L74
.L81:
	.align	2
.L80:
	.word	.LANCHOR0
	.word	analog_readings
	.size	robot_fill_packet, .-robot_fill_packet
	.bss
	.align	2
	.set	.LANCHOR0,. + 0
	.type	top_claw, %object
	.size	top_claw, 32
top_claw:
	.space	32
	.type	bottom_claw, %object
	.size	bottom_claw, 32
bottom_claw:
	.space	32
	.type	value7.2464, %object
	.size	value7.2464, 4
value7.2464:
	.space	4
	.type	pusher_distal, %object
	.size	pusher_distal, 8
pusher_distal:
	.space	8
	.type	value0.2462, %object
	.size	value0.2462, 4
value0.2462:
	.space	4
	.type	pusher_distal_captures, %object
	.size	pusher_distal_captures, 8
pusher_distal_captures:
	.space	8
	.type	pusher_proximal, %object
	.size	pusher_proximal, 8
pusher_proximal:
	.space	8
	.type	pusher_proximal_captures, %object
	.size	pusher_proximal_captures, 8
pusher_proximal_captures:
	.space	8
	.type	value6.2463, %object
	.size	value6.2463, 4
value6.2463:
	.space	4
	.type	ultrasonic_length, %object
	.size	ultrasonic_length, 4
ultrasonic_length:
	.space	4
	.ident	"GCC: (GNU) 4.5.4"
